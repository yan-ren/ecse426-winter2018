'use strict';
const functions = require('firebase-functions');
const gcs = require('@google-cloud/storage')();
const speech = require('@google-cloud/speech');
const path = require('path');
const os = require('os');
const fs = require('fs');
const linear16 = require('linear16');
const delay = require('delay');
var http = require('http');
const ffmpeg = require('fluent-ffmpeg');
var ffmpegBinaries = require("ffmpeg-binaries")
var ffmpegPath = ffmpegBinaries.ffmpegPath() // Path to ffmpeg binary
var ffprobePath = ffmpegBinaries.ffprobePath() // Path to ffprobe binary
const bucket = gcs.bucket('microp-d55c0.appspot.com');
const gcsUri = 'gs://microp-d55c0.appspot.com/Sound/track.wav';
const gcsUriRes = 'gs://microp-d55c0.appspot.com/res.txt';
var uint8ToStr = require('arraybuffer-to-string');
var tmp = require('tmp');
var hexToDec = require('hex-to-dec');
var hexString = require('hex-string');
var plotly = require('plotly')("grantzhao75", "H2GCVEothMxnUJWBOAKB");
var toArray = require('stream-to-array');
exports.convertToText = //trigger
	functions.storage.object().onFinalize((object) => {  //detect when a txt file is created on cloud storage
		//event attributes
		const fileBucket = object.bucket; // The Storage bucket that contains the file.
		const filePath = object.name; // File path in the bucket.
		const contentType = object.contentType; // File content type.
		const resourceState = object.resourceState; // The resourceState is 'exists' or 'not_exists' (for file/folder deletions).
		const metageneration = object.metageneration; // Number of times metadata has been generated. New objects have a value of 1.
		const fileName = path.basename(filePath);

		if (contentType.startsWith('audio/')) {
			console.log('This is an audio file');
			//convert();
			readAudio('gs://microp-d55c0.appspot.com/Sound/track.wav','LINEAR16',10000);
			//api demo
			readAudio('gs://microp-d55c0.appspot.com/eg_006_sd.flac','FLAC',16000);
			return null;
		}		 		

		if (contentType.startsWith('text/')) {
			console.log('This is an text file');
			readArray('microp-d55c0.appspot.com', 'pitch.txt','roll.txt', plot);
			delay(20000)
				    .then(() => {
						downloadImg();
				        return null;
				    }).catch(err => {
					    console.error('ERROR:', err);
					  });
		}		 
		return null;
	});


function downloadImg() {
	var https = require('https');
	console.log('downloading img...');
	var file = bucket.file('0.jpeg').createWriteStream();
	var request = https.get('https://plot.ly/~grantzhao75/0.jpeg', function(response) {
	  response.pipe(file);
	});
}


var readArray = function(bucketName,fileName1,fileName2,callback){
	var fs = require('fs');
	const os = require('os');
	//read txt file
	const tempFilePath1 = path.join(os.tmpdir(), fileName1);
	const tempFilePath2 = path.join(os.tmpdir(), fileName2);
	console.log('tempFilePath1 is:',tempFilePath1);
	console.log('tempFilePath2 is:',tempFilePath2);
	//download signal to temp
	downloadFile(bucketName, fileName1, tempFilePath1);
	downloadFile(bucketName, fileName2, tempFilePath2);
	delay(20000)
	    .then(() => {
			var array1 = fs.readFileSync(tempFilePath1).toString().split(",");
			var arrayLen1 = array1.length;
			console.log('arraylength1:' + arrayLen1);
			var array2 = fs.readFileSync(tempFilePath2).toString().split(",");
			var arrayLen2 = array2.length;
			console.log('arraylength2:' + arrayLen2);
	        //return [array, arrayLen];
	        // return {
	        // 	array: array,
	        // 	arrayLen: arrayLen
	        // };
	        callback(array1,array2,arrayLen1,arrayLen2);
	        return null;
	    }).catch(err => {
		    console.error('ERROR:', err);
		  });
}
function plot(array1,array2,arrayLen1,arrayLen2) {

			console.log('in plot');

			var pitch = array1;
			var pitchLen = arrayLen1;
			var roll = array2;
			var rollLen = arrayLen2;			
			
			var x_p = [];
			var x_r = [];
			//retrieve pitch value
			var i;	
			for (i=0; i<pitchLen; i++) {
				pitch[i] = pitch[i] - 90;
			}
			var j;
			for (j=0; j<pitchLen; j++) {
				x_p[j] = j;
			}
			//retrieve roll value
			var k;	
			for (k=0; k<rollLen; k++) {
				roll[k] = roll[k] - 90;
			}
			var l;
			for (l=0; l<rollLen; l++) {
				x_r[l] = l;
			}

			var pitchPlot = {
			  x: x_p,
			  y: pitch,
			  type: "scatter"
			};
			var rollPlot = {
			  x: x_r,
			  y: roll,
			  type: "scatter"
			};
			var data = [pitchPlot, rollPlot];
			var graphOptions = {filename: "basic-line", fileopt: "overwrite"};
			plotly.plot(data, graphOptions, function (err, msg) {
			    console.log(msg);
			    if(err) throw err;
			});
		        return null;

}
function convert() {

	const os = require('os');
	//read txt file
	const tempFilePath = path.join(os.tmpdir(), 'track.wav');
	console.log('tempFilePath is:',tempFilePath);
	//download signal to temp
	downloadFile('microp-d55c0.appspot.com', 'track.wav', tempFilePath);

	try {
		new ffmpeg(tempFilePath, function (err, audio) {
			if (!err) {
				console.log('The audio is ready to be processed');
			} else {
				console.log('Error: ' + err);
			}
		});
	} catch (e) {
		console.log(e.code);
		console.log(e.msg);
	}
	linear16('/tmp/track.wav', 'gs://microp-d55c0.appspot.com/ConvertedTrack.wav')
	//write the result to the temp file
	return null;
}

function readAudio(uri,encode,spRate) {
		const speech = require('@google-cloud/speech');
		//start conversion
		const client = new speech.SpeechClient();
		const config = {
			encoding: encode,
			sampleRateHertz: spRate,
			languageCode: 'en-US',
		};

		const audio = {
			uri: uri,
		};

		const request = {
			config: config,
			audio: audio,
		};
		// Detects speech in the audio file
		client
		  .recognize(request)
		  .then(data => {
		    const response = data[0];
		    const transcription = response.results
		      .map(result => result.alternatives[0].transcript)
		      .join('\n');
		    console.log(`Audio uploaded: `, transcription);
		    //printToFile(transcription);
		    return null;
		  })
		  .catch(err => {
		    console.error('ERROR:', err);
		  });
}

function printToFile(res) {
	
	const os = require('os');
	//read txt file
	const tempFilePath = path.join(os.tmpdir(), 'res.txt');
	console.log('tempFilePath is:',tempFilePath);
	//download signal to temp
	downloadFile('microp-d55c0.appspot.com', 'res.txt', tempFilePath);

	//write the result to the temp file
	fs.writeFile('/tmp/res.txt', res, (err) => {
        if(err) throw err;
		//upload result txt to coloud storage
		uploadFile('microp-d55c0.appspot.com','/tmp/res.txt');			    
	    console.log('The result file was saved to cloud storage');	
	return null;
	});	
	return null;
}

function uploadFile(bucketName, filename) {


  // Creates a client
  const storage = new Storage();

  storage
    .bucket(bucketName)
    .upload(filename)
    .then(() => {
      console.log(`${filename} uploaded to ${bucketName}.`);
      return null;
    })
    .catch(err => {
      console.error('ERROR:', err);
    });
}

function downloadFile(bucketName, srcFilename, destFilename) {
  const Storage = require('@google-cloud/storage');

  // Creates a client
  const storage = new Storage();

  const options = {
    // The path to which the file should be downloaded, e.g. "./file.txt"
    destination: destFilename,
  };

  // Downloads the file
  storage
    .bucket(bucketName)
    .file(srcFilename)
    .download(options)
    .then(() => {
      console.log(
        `gs://${bucketName}/${srcFilename} downloaded to ${destFilename}.`
      );
      return null;
    })
    .catch(err => {
      console.error('ERROR:', err);
    });
  // [END storage_download_file]
}
