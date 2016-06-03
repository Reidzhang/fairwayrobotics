// Connecting to ROS
// -----------------

var ros = new ROSLIB.Ros({
	// url : 'ws://loggerhead.cs.washington.edu:9090'
	// url : 'ws://localhost:9090'
	url : 'ws://chester.cs.washington.edu:9090'
});

ros.on('connection', function() {
	console.log('Connected to websocket server.');
});

		ros.on('error', function(error) {
	console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
	console.log('Connection to websocket server closed.');
});

// Subscribing to a Topic
// ----------------------

var stationLocationPublisher = new ROSLIB.Topic({
	ros : ros,
	name : '/choose_station',
	messageType : 'std_msgs/Int8'
	});

var ballRequestPublisher = new ROSLIB.Topic({
  ros : ros,
  name : '/request_balls',
  messageType : 'std_msgs/Empty'
  });

var finishedPublisher = new ROSLIB.Topic({
  ros : ros,
  name : '/finished',
  messageType : 'std_msgs/Empty'
});

var donePublisher = new ROSLIB.Topic({
  ros : ros,
  name : '/done',
  messageType : 'std_msgs/Empty'
});

var backToMainSubscriber = new ROSLIB.Topic({
  ros : ros,
  name : '/at_station',
  messageType : 'std_msgs/Empty'
});

var pleaseMoveSubscriber = new ROSLIB.Topic({
  ros : ros,
  name : '/please_move',
  messageType : 'std_msgs/Empty'
});

var missionAbortedSubscriber = new ROSLIB.Topic({
  ros : ros,
  name : '/mission_abort',
  messageType : 'std_msgs/Empty'
});

var pleaseMoveSuccessSubscriber = new ROSLIB.Topic({
  ros : ros,
  name : '/please_move_success',
  messageType : 'std_msgs/Empty'
});

var stationNumber;
var basketCount;

window.onload = function() {
  // Add button onclick listeners
	var stationButtons = document.getElementsByClassName("stationButton");

	for (var i = 0; i < stationButtons.length; i++) {
  	stationButtons[i].addEventListener('click', function() {
      stationNumber = this.getAttribute('value');
      basketCount = 0;

    	toMovingPageFromIndex();
  	}, false);
	}

  addClickListener("requestButton", toMovingPageFromMain);
  addClickListener("finishButton", toFinishPage);
  addClickListener("doneButton", toIndexPageFromFinish);
  addClickListener("restartButton", toIndexPageFromAbort);
  addClickListener("mainPageButton", toMainPageFromMove);
}

var addClickListener = function(id, callback) {
  document.getElementById(id).addEventListener('click', callback, false);
}

var toMovingPageFromIndex = function() {
  toMovingPage('indexPage');
}

var toMovingPageFromMain = function() {
  toMovingPage('mainPage');
}

var toIndexPageFromFinish = function() {
  toIndexPage('finishPage');
}

var toIndexPageFromAbort = function() {
  toIndexPage('missionAbortPage');
}

var toMovingPage = function(sourcePage) {
  // Publish ball requested message
  if (sourcePage === 'mainPage') {
    var ballRequestMessage = new ROSLIB.Message({});
    ballRequestPublisher.publish(ballRequestMessage);
  } else if (sourcePage === 'indexPage') {
    var stationLocationMessage = new ROSLIB.Message({
      data: parseInt(stationNumber)
    });
    stationLocationPublisher.publish(stationLocationMessage); 
    
    // Set main page text for station number and number baskets used
    var stationText = document.getElementById('mainStationNumberText');
    stationText.innerHTML = 'Station ' + stationNumber;

    var basketNumText = document.getElementById('mainBasketCountText');
    basketNumText.innerHTML = 'Basket ' + "( " + basketCount + " )";
  }

  // Set up subscriber to listen for message that
  // robot is back at station
	// TO DO: Move it somewhere else
  backToMainSubscriber.subscribe(function(message) {
    console.log('Received message on ' + backToMainSubscriber.name + ': ' + message.data);
    toMainPageFromMove();
    backToMainSubscriber.unsubscribe();
    pleaseMoveSubscriber.unsubscribe();
    missionAbortedSubscriber.unsubscribe();
  });

  // Set up subscriber to listen for message that
  // person may be blocking robot
  pleaseMoveSubscriber.subscribe(function(message) {
    console.log('Received message on ' + pleaseMoveSubscriber.name + ': ' + message.data);
    toPleaseMovePage();
    backToMainSubscriber.unsubscribe();
    pleaseMoveSubscriber.unsubscribe();
    missionAbortedSubscriber.unsubscribe();
  });

  // Set up subscriber to listen for message that
  // mission has been aborted
  missionAbortedSubscriber.subscribe(function(message) {
    console.log('Received message on ' + missionAbortedSubscriber.name + ': ' + message.data);
    toMissionAbortPage();
    backToMainSubscriber.unsubscribe();
    pleaseMoveSubscriber.unsubscribe();
    missionAbortedSubscriber.unsubscribe();
  });
  
  // Hide main page, show moving page
  document.getElementById(sourcePage).style.display = 'none';
  document.getElementById("movingPage").style.display = 'inline';
};

var toFinishPage = function() {
  // Publish finished message
  var finishedMessage = new ROSLIB.Message({});
  finishedPublisher.publish(finishedMessage);
  
  // Set finish page text for station number and number baskets used
  var stationText = document.getElementById('finishStationNumberText');
  stationText.innerHTML = 'Station Number: ' + stationNumber;

  var basketText = document.getElementById('finishBasketCountText');
  basketText.innerHTML = 'Baskets Used: ' + basketCount;

  // Hide main page, show finish page
  document.getElementById("mainPage").style.display = 'none';
  document.getElementById("finishPage").style.display = 'inline';
};

var toIndexPage = function(sourcePage) {
  if (sourcePage == 'finishPage') {
    // Publish done message
    var doneMessage = new ROSLIB.Message({});
    donePublisher.publish(doneMessage);
  }
  
  // Hide finish page, show index page
  document.getElementById(sourcePage).style.display = 'none';
  document.getElementById("indexPage").style.display = 'inline';
};

var toMainPageFromMove = function() {
  // Update number of baskets used
  basketCount++;

  // Update main page text to show new number of baskets used
  var basketNumText = document.getElementById('mainBasketCountText');
  basketNumText.innerHTML = 'Basket ' + "( " + basketCount + " )";

  // Hide moving page, show main page
  document.getElementById("movingPage").style.display = 'none';
  document.getElementById("mainPage").style.display = 'inline';
};

var toPleaseMovePage = function() {
  // Set up subscriber to listen for message that
  // obstacle has moved out of the way
  pleaseMoveSuccessSubscriber.subscribe(function(message) {
    console.log('(!!)Received message on ' + pleaseMoveSuccessSubscriber.name + ': ' + message.data);
    toMovingPage('pleaseMovePage');
    pleaseMoveSuccessSubscriber.unsubscribe();
  });

	console.log('Changing page');

  // Hide moving page, show please move page
  document.getElementById("movingPage").style.display = 'none';
  document.getElementById("pleaseMovePage").style.display = 'inline';
};

var toMissionAbortPage = function() {
  // Hide moving page, show mission abort page
  document.getElementById("movingPage").style.display = 'none';
  document.getElementById("missionAbortPage").style.display = 'inline';
};
