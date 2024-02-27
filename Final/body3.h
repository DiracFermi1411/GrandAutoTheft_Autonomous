const char body[] PROGMEM = R"===(
<!DOCTYPE html>  
  <html>
  <head>
    <style>
      th, td {
        border-style: outset;
      }
      table {
        width: 100%;
        font-family: Arial, Helvetica, sans-serif;
      }
      td {
        text-align: center;
        height: 200px;
        font-size: 110px;
      }
      .button {
        background-color: transparent;
        border: none;
        color: black;
        //padding: 15px 32px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 100px;
        margin: 0px 0px;
        cursor: pointer;
        border-radius: 4px;
        width: 100%;
        height: 250px;
        outline:none;
        transition-duration: 0.1s;
      }
      .button:hover {
        background-color: #e7e7e7;
      }
      .slider {
        -webkit-appearance: none;
        width: 100%;
        height: 100px;
        background: #d3d3d3;
        outline: none;
        opacity: 0.7;
        -webkit-transition: .2s;
        transition: opacity .2s;
      }

      .slider:hover {
        opacity: 1;
      }

      .slider::-webkit-slider-thumb {
        -webkit-appearance: none;
        appearance: none;
        width: 100px;
        height: 100px;
        background: #04AA6D;
        cursor: pointer;
      }

      .slider::-moz-range-thumb {
        width: 25px;
        height: 25px;
        background: #04AA6D;
        cursor: pointer;
      }
      .switch {
        position: relative;
        display: inline-block;
        width: 100%;
        height: 200px;
      }

      .switch input { 
        opacity: 0;
        width: 0;
        height: 0;
      }

      body {
        display: flex;
        justify-content: center;
        align-items: center;
        height: 100vh;
        margin: 0;
      }
      #location {
        font-size: 24px;
      }
    </style>
  </head>
    <body>
      <h1>  </h1>
      <table>
        <tr>
          <td colspan="3">Controller</td>
        </tr>
        <tr>
          <td><button type="button" class="button" onclick="onoff1(this)">w</button></td>
          <td><button type="button" class="button" onmousedown="hit1(this)" onmouseup="hit5(this)" ontouchstart="hit1(this)" ontouchend="hit5(this)"></button></td>
          <td><button type="button" class="button" onclick="onoff2(this)">R</button></td>
        </tr>
        <tr>
          <td><button type="button" class="button" onmousedown="hit4(this)" onmouseup="hit5(this)" ontouchstart="hit4(this)" ontouchend="hit5(this)"></button></td>
          <td><button type="button" class="button" onclick="hit5(this)">S</button></td>
          <td><button type="button" class="button" onmousedown="hit3(this)" onmouseup="hit5(this)" ontouchstart="hit3(this)" ontouchend="hit5(this)"></button></td>
        </tr>
        <tr>
          <td><button type="button" class="button" onclick="onoff3(this)">Y</button></td>
          <td><button type="button" class="button" onmousedown="hit2(this)" onmouseup="hit5(this)" ontouchstart="hit2(this)" ontouchend="hit5(this)"></button></td>
          <td><button type="button" class="button" onclick="onoff4(this)">P</button></td>
        </tr>
        // <tr>
        //   <td colspan="3"><span id="outputXY"></span> <br><br></td>
        // </tr>
        <tr>
          <td colspan="3"><span id="outputlabelDuty">0</span> <br></td>
        </tr>
        <tr>
          <td colspan="3"><input class="slider" type="range" min="0" max="100" value="0" id="duty" onchange="updateDutyLabel(this.value)">
        </td>
        </tr>
      </table>

      <div id="location">Loading location...</div>

      
    </body>

    <script>
      function updateDutyLabel(value) {
        document.getElementById("outputlabelDuty").innerHTML = value;
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
            if (this.readyState === 4 && this.status === 200) {
                document.getElementById("outputlabelDuty").innerHTML = this.responseText;
            }
        };
        var str = "duty?val==" + value;
        xhttp.open("GET", str, true);
        xhttp.send();
      }

      function hit1(obj) {
        obj.style.backgroundColor = "#04AA6D";
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitF", true);
        xhttp.send();
      }

      function hit2(obj) {
        obj.style.backgroundColor = "#04AA6D";
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitB", true);
        xhttp.send();
      }

      function hit3(obj) {
        obj.style.backgroundColor = "#04AA6D";
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitR", true);
        xhttp.send();
      }

      function hit4(obj) {
        obj.style.backgroundColor = "#04AA6D";
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitL", true);
        xhttp.send();
      }

      function hit5(obj) {
        obj.style.backgroundColor="transparent";
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitS", true);
        xhttp.send();
      }

      function hit6(obj) {
        obj.style.backgroundColor="#e7e7e7";
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitS", true);
        xhttp.send();
      }

      function hit7(obj) {
        obj.style.backgroundColor="#e7e7e7";
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitW", true);
        xhttp.send();
      }

      function hit8(obj) {
        obj.style.backgroundColor="#e7e7e7";
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitT", true);
        xhttp.send();
      }

      function hit9(obj) {
        obj.style.backgroundColor="#e7e7e7";
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "hitTT", true);
        xhttp.send();
      }

      // function hit10(obj) {
      //   var xhttp = new XMLHttpRequest();
      //   var str = "hitC";
      //   var outputElement = document.getElementById("outputXY");
      //   outputElement.innerHTML = 'X: ' + x_relative.x + ', Y: ' + y_relative.y;
      //   var res = str.concat(x_relative,",",y_relative);
      //   xhttp.onreadystatechange = function() {
      //     if (this.readyState == 4 && this.status == 200) {
      //       // Assuming you want to display the server response as well
      //       outputElement.innerHTML += '<br>Server Response: ' + this.responseText;
      //     }
      //   };
      //   xhttp.open("GET", res, true);
      //   xhttp.send();
      // }

      var buttonstate1=0;
      function onoff1(obj)
      {
        buttonstate1= 1 - buttonstate1;
        var blabel, bstyle, bcolor;
        if(buttonstate1)
        {
          obj.style.backgroundColor="#04AA6D";
          var xhttp = new XMLHttpRequest();
          xhttp.open("GET", "hitW", true);
          xhttp.send();
        }
        else
        {
          obj.style.backgroundColor="#e7e7e7";
          var xhttp = new XMLHttpRequest();
          xhttp.open("GET", "hitS", true);
          xhttp.send();
        }
      }
      
      var buttonstate2=0;
      function onoff2(obj)
      {
        buttonstate2= 1 - buttonstate2;
        var blabel, bstyle, bcolor;
        if(buttonstate2)
        {
          obj.style.backgroundColor="#04AA6D";
          var xhttp = new XMLHttpRequest();
          xhttp.open("GET", "hitT", true);
          xhttp.send();
        }
        else
        {
          obj.style.backgroundColor="#e7e7e7";
          var xhttp = new XMLHttpRequest();
          xhttp.open("GET", "hitS", true);
          xhttp.send();
        }
        
      }
      
      var buttonstate3=0;
      function onoff3(obj)
      {
        buttonstate3= 1 - buttonstate3;
        if(buttonstate3)
        {
          obj.style.backgroundColor="#04AA6D";
          var xhttp = new XMLHttpRequest();
          xhttp.open("GET", "hitTT", true);
          xhttp.send();
        }
        else
        {
          obj.style.backgroundColor="#e7e7e7";
          var xhttp = new XMLHttpRequest();
          xhttp.open("GET", "hitS", true);
          xhttp.send();
        }
      }
      
      var buttonstate4=0;
      function onoff4(obj)
      {
        buttonstate4= 1 - buttonstate4;
        if(buttonstate4)
        {
          obj.style.backgroundColor="#04AA6D";
          var xhttp = new XMLHttpRequest();
          xhttp.open("GET", "hitP", true);
          xhttp.send();
        }
        else
        {
          obj.style.backgroundColor="#e7e7e7";
          var xhttp = new XMLHttpRequest();
          xhttp.open("GET", "hitS", true);
          xhttp.send();
        }
      }

      // Set up a WebSocket connection to Arduino
      // const socket = new WebSocket('ws://192.168.1.102:14');

      // Update location when receiving data from Arduino
      // socket.addEventListener('message', (event) => {
      //   const data = JSON.parse(event.data);
      //   const locationElement = document.getElementById('location');
      //   locationElement.innerText = `X: ${data.X}, Y: ${data.Y}`;
      // });
    </script>   
  </html>  
)===";
