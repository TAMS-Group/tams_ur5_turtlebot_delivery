var ros;
var handStateListener;
var compressedImageListener;

//@deprecated
function testConnect(){
	// handling the connection
	ros = new ROSLIB.Ros({url : 'ws://tams43:9090'});

	ros.on('connection', function() {
		console.log('Connected to websocket server.')
	});

	ros.on('error', function(error) {
		console.log('Error connecting to websocket server: ', error)
	});

	ros.on('close', function() {
		console.log('Connection to websocket server closed.')
	});

	// publishing a topic
	var cmdVel = new ROSLIB.Topic({
		ros : ros,
		id : '/cmd_vel',
		messageType : 'geometry_msgs/Twist'
	});

	var twist = new ROSLIB.Message({
		linear : {
			x : 0.1,
			y : 0.2,
			z : 0.3
		},
		angular : {
			x : -0.1,
			y : -0.2,
			z : -0.3
		}
	});

	cmdVel.publish(twist);
}

function connect(){
	// handling the connection
	ros = new ROSLIB.Ros({url : 'ws://tams43:9090'});

	ros.on('connection', function() {
		console.log('Connected to websocket server.')
	});

	ros.on('error', function(error) {
		console.log('Error connecting to websocket server: ', error)
	});

	ros.on('close', function() {
		console.log('Connection to websocket server closed.')
	});
}

//activates the hand
function initHand(){
	// publishing a topic
	var outputMessage = new ROSLIB.Topic({
		ros : ros,
		name : '/SModelRobotOutput',
		messageType : 'robotiq_s_model_control/SModel_robot_output'
	});

	var smodelControl = new ROSLIB.Message({
		rACT : 1,
		rMOD : 0,
		rGTO : 1,
		rATR : 0,
		rGLV : 0,
		rICF : 0,
		rICS : 0,
		rPRA : 0,
		rSPA : 255,
		rFRA : 150,
		rPRB : 0,
		rSPB : 0,
		rFRB : 0,
		rPRC : 0,
		rSPC : 0,
		rFRC : 0,
		rPRS : 0,
		rSPS : 0,
		rFRS : 0,
	});

	outputMessage.publish(smodelControl);
}

//fully opens the hand
function openHand(){
	// publishing a topic
	var outputMessage = new ROSLIB.Topic({
		ros : ros,
		name : '/SModelRobotOutput',
		messageType : 'robotiq_s_model_control/SModel_robot_output'
	});

	var smodelControl = new ROSLIB.Message({
		rACT : 1,
		rMOD : 0,
		rGTO : 1,
		rATR : 0,
		rGLV : 0,
		rICF : 0,
		rICS : 0,
		rPRA : 0,
		rSPA : 255,
		rFRA : 150,
		rPRB : 0,
		rSPB : 0,
		rFRB : 0,
		rPRC : 0,
		rSPC : 0,
		rFRC : 0,
		rPRS : 0,
		rSPS : 0,
		rFRS : 0,
	});

	outputMessage.publish(smodelControl);
}

//fully closes the hand
function closeHand(){
	// publishing a topic
	var outputMessage = new ROSLIB.Topic({
		ros : ros,
		name : '/move_base_simple/goal',
		messageType : 'geometry_msgs/PoseStamped'
	});

	var smodelControl = new ROSLIB.Message(
		{
			header: {frame_id: "map"},
			pose: {
				position: {x: 4.5, y: 3.4, z: 0.0},
				orientation: {z: 2}
		}
		});

	outputMessage.publish(smodelControl);
}

function unsubscribeHandState(){
	handStateListener.unsubscribe();
}

function getCompressedImage(){ // ~30 fps
	compressedImageListener = new ROSLIB.Topic({
		ros : ros,
		//name : '/camera/rgb/image_rect_color/compressed',
		name : '/camera/rgb/image_raw/compressed',
		//name : '/camera/ir/image/compressed',
		//name : '/camera/depth/points',
		messageType : 'sensor_msgs/CompressedImage'
	});

	compressedImageListener.subscribe(function(message){
		//console.log('Received image ' + compressedImageListener.id + ': '
			//+' format: '
			//+message.format
			//+' HEADER: '
			//+' seq: '
			//+message.header.seq
			//+' timestamp: '
			//+message.header.stamp.sec+":"+message.header.stamp.nsec
			//+' frameId '
			//+message.header.frame_id
			//+' '
			//+message.data
		//);
		//console.log(message);
		draw(message.data)			
	});
}

function unsubscribeCompressedImage(){
	compressedImageListener.unsubscribe();
}

function draw(imgData) {
    "use strict";
    var canvas = document.getElementById("thecanvas");
    var ctx = canvas.getContext("2d");

    var uInt8Array = imgData;
    var bild = ""+uInt8Array;
  
    var img = new Image();
    img.src = "data:image/jpeg;base64," + bild;
  
    img.onload = function () {
        console.log("Image Onload");
        ctx.drawImage(img, 0, 0);
    };
    img.onerror = function (stuff) {
        console.log("Img Onerror:", stuff);
	};
}