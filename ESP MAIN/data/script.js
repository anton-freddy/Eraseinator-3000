var gateway = `ws://${window.location.hostname}/ws`;
var websocket;
var robotDot = null;

window.addEventListener('load', onLoad);
function initWebSocket() {
    console.log('Trying to open a WebSocket connection...');
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
}
function onOpen(event) {
    console.log('Connection opened');
}

function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}

function onMessage(event) {
    var xValue
    var yValue
    var xRobot
    var yRobot

    var datatosplit = event.data;
    var datasplit = datatosplit.split(',');

    xValue = parseFloat(datasplit[0]);
    yValue = parseFloat(datasplit[1]);
    xRobot = parseFloat(datasplit[2]);
    yRobot = parseFloat(datasplit[3]);

    updateRobotPosition(xRobot, yRobot);
    addCord(xValue, yValue, '#0000ff');

}




function createDot(x, y, color) {
    var dot = document.createElement('div');
    dot.style.position = 'absolute';
    dot.style.width = '5px';
    dot.style.height = '5px';
    dot.style.borderRadius = '50%';
    dot.style.backgroundColor = color;

    dot.style.left = (x * 4 * scaleFactor + xyPlot.offsetWidth / 2) + 'px';
    dot.style.top = (y * 4 * scaleFactor + xyPlot.offsetHeight / 2) + 'px';

    return dot;
}

function updateRobotPosition(x, y) {
    // Remove the previous robot position dot
    if (robotDot) {
        xyPlot.removeChild(robotDot);
    }

    // Create a new robot position dot
    robotDot = createDot(x, y, '#ff0000');
    document.getElementById('xyPlane').appendChild(robotDot);
}

function addCord(x, y, color) {
    var dot = createDot(x, y, color);
    document.getElementById('xyPlane').appendChild(dot);
}