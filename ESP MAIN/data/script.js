function sendData() {
    var xValue = document.getElementById('inputX').value;
    var yValue = document.getElementById('inputY').value;
    var data = xValue + ',' + yValue;
    webSocket.send(data);

  }
  
  var webSocket = new WebSocket(`ws://${window.location.hostname}/ws`);
  webSocket.onopen = function (event) {
    console.log('WebSocket connection opened.');
};
  var xyPlane = document.getElementById('xyPlane');
  
  webSocket.onmessage = function (event) {
    var logBox = document.getElementById('logBox');
    logBox.innerHTML += '<div>' + event.data + '</div>';
    logBox.scrollTop = logBox.scrollHeight;
  
    var datatosplit = event.data;
    var datasplit = datatosplit.split(',');
    console.log('Received ', datasplit);
  
    var xValue = parseFloat(datasplit[0]);
    var yValue = parseFloat(datasplit[1]);
    console.log('Received X:', xValue, 'Y:', yValue);
  
    var dot = document.createElement('div');
    dot.style.position = 'absolute';
    dot.style.width = '5px';
    dot.style.height = '5px';
    dot.style.borderRadius = '50%';
    dot.style.backgroundColor = '#FF0000';
  
    // Adjust the scaling factor based on the size of the xyPlane
    var scaleFactor = xyPlane.offsetWidth / 800; // Assuming the xyPlane is 800x800 pixels
  
    dot.style.left = (xValue * 4 * scaleFactor + xyPlane.offsetWidth / 2) + 'px';
    dot.style.top = (-yValue * 4 * scaleFactor + xyPlane.offsetHeight / 2) + 'px'; // Note the negative sign for the y-axis
    xyPlane.appendChild(dot);
  };
  