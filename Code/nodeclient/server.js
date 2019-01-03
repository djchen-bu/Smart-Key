var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var Engine = require('tingodb')();
var db = new Engine.Db('./database', {});
var events = require('events');
var eventEmitter = new events.EventEmitter();
var httpserver = require('http');

const request = require('request');

var fobid = 0;
var hubid = 0;
var msg;
var code = 0x69;

var tempip;

var val1;
var val2;
var val3;

var timeout = 10;

var keyip1 = 'http://192.168.1.148';
var keyip2 = 'http://192.168.1.148';
var keyip3 = 'http://192.168.1.148';
var secip0 = 'http://192.168.1.126';

var name1 = 'Devin';
var name2 = 'Carlos';
var name3 = 'Mahdiul';

var keystatus1 = 0;
var keystatus2 = 0;
var keystatus3 = 0;

var tcount1;
var tcount2;
var tcount3;

var dbindex1;
var dbindex2;
var dbindex3;

var date = new Date();
var timestamp = date.getTime();

db.createCollection('KeyfobData');
var collection = db.collection('KeyfobData');

//sends over all objects with ids 1, 2, 3
function getHistory(){
	var i;
	collection.find({fobId:"1"}).toArray(function(err, result) {
	    if (err) throw err;
	    dbindex1 = result.length;
	    for(i = 0; i < dbindex1; i++){
		    var dataobj = {[dbindex1]:[result[i]]};
		    io.emit('fob1history',dataobj);
		}
	});

	collection.find({fobId:"2"}).toArray(function(err, result) {
	    if (err) throw err;
	    dbindex2 = result.length;
	    for(i = 0; i < dbindex2; i++){
		    var dataobj = {[dbindex2]:[result[i]]};
		    io.emit('fob2history',dataobj);
		}
	});

	collection.find({fobId:"3"}).toArray(function(err, result) {
	    if (err) throw err;
	    dbindex3 = result.length;
	    for(i = 0; i < dbindex3; i++){
		    var dataobj = {[dbindex3]:[result[i]]};
		    io.emit('fob3history',dataobj);
		}
	});
	console.log("done");
}

//runs in the beginning to populate history
function initalhistory(){
	getHistory();
	clearInterval(inithis);
}

//delays 10 seconds before populating history
var inithis = setInterval(initalhistory, 10000);

//timeout functions which reset and send led shut off to the fob
function timeout1(){
	tcount1 -= 1;
	if(tcount1 == 0){
		keystatus1 = 0;
		tcount1 = timeout;
		date = new Date();
		timestamp = date.getTime();
		collection.insert({Timestamp:timestamp, fobId:fobid, hubId:hubid, Name:name1, Status:keystatus1});
		fobid = 0;
		getHistory();
		io.emit('fobstatus1', '0');
		request.post(keyip1 + '/fob_post').form({0:'0'});
		clearInterval(val1);
	}
}

function timeout2(){
	tcount2 -= 1;
	if(tcount2 == 0){
		keystatus2 = 0;
		tcount2 = timeout;
		date = new Date();
		timestamp = date.getTime();
		collection.insert({Timestamp:timestamp, fobId:fobid, hubId:hubid, Name:name2, Status:keystatus2});
		fobid = 0;
		getHistory();
		io.emit('fobstatus2', '0');
		request.post(keyip2 + '/fob_post').form({0:'0'});
		clearInterval(val2);
	}
}

function timeout3(){
	tcount3 -= 1;
	if(tcount3 == 0){
		keystatus3 = 0;
		tcount3 = timeout;
		date = new Date();
		timestamp = date.getTime();
		collection.insert({Timestamp:timestamp, fobId:fobid, hubId:hubid, Name:name3, Status:keystatus3});
		fobid = 0;
		getHistory();
		io.emit('fobstatus3', '0');
		request.post(keyip3 + '/fob_post').form({0:'0'});
		clearInterval(val3);
	}
}

//handlers on receiving data
var fob1Handler = function() {
	console.log('This is fobid 1');
	keystatus1 = 1;
	tcount1 = timeout;
	date = new Date();
	timestamp = date.getTime();
	collection.insert({Timestamp:timestamp, fobId:fobid, hubId:hubid, Name:name1, Status:keystatus1});
	getHistory();
	val1 = setInterval(timeout1, 1000);
}

var fob2Handler = function() {
	console.log('This is fobid 2');
	keystatus2 = 1;
	tcount2 = timeout;
	date = new Date();
	timestamp = date.getTime();
	collection.insert({Timestamp:timestamp, fobId:fobid, hubId:hubid, Name:name2, Status:keystatus2});
	getHistory();
	val2 = setInterval(timeout2, 1000);
}

var fob3Handler = function() {
	console.log('This is fobid 3');
	keystatus3 = 1;
	tcount3 = timeout;
	date = new Date();
	timestamp = date.getTime();
	collection.insert({Timestamp:timestamp, fobId:fobid, hubId:hubid, Name:name3, Status:keystatus3});
	getHistory();
	val3 = setInterval(timeout3, 1000);
}

//emitters setup on post receive
eventEmitter.on('1', fob1Handler);
eventEmitter.on('2', fob2Handler);
eventEmitter.on('3', fob3Handler);

//http server to receive post requests
var server = httpserver.createServer ( function(req,res, body){

    res.writeHead(200,{"Content-Type":"text\plain"});
    if(req.method == "GET")
        {
            res.end("received GET request.");
            console.log("received GET request.");
        }
    else if(req.method == "POST")
        {
            res.end("received POST request.");
            let body = '';
		    req.on('data', chunk => {
		        body += chunk.toString(); // convert Buffer to string
		    });
		    req.on('end', () => {
		        console.log(body);
		        res.end('ok');
		        
		        msg = body.split(",")
		        if(msg[0] == code && msg[1] != fobid && msg[1] != 0){
		        	fobid = msg[1];
		        	hubid = msg[2];
		        	eventEmitter.emit(fobid);
		        	if(fobid == 1){
		        		io.emit('fobstatus1', '1');
		        		request.post(keyip1 + '/fob_post').form({1:'1'});
		        	}else if(fobid == 2){
		        		io.emit('fobstatus2', '1');
		        		request.post(keyip2 + '/fob_post').form({1:'1'});
		        	}else if(fobid == 3){
		        		io.emit('fobstatus3', '1');
		        		request.post(keyip3 + '/fob_post').form({1:'1'});
		        	}else
		        		console.log("Invalid Id")
		        }
		        
		    });
        }
    else
        {
            res.end("Undefined request .");
        }
});

server.listen(8000);
console.log("Server running on port 8000");

// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
});

// User socket connection
io.on('connection', function(socket){
  console.log('a user connected');
  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
});

// Listening on localhost:3000

http.listen(3000, function() {
  console.log('listening on *:3000');
});
