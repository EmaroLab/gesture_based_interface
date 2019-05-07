  ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

  // Publishing in a Topic
  // ------------------
  var selection = new ROSLIB.Topic({
    ros : ros,
    name : '/http_server_selection',
    messageType : 'http_bridge_msgs/selection'
  });

  function getPlatformType() {
    if(navigator.userAgent.match(/mobile/i)) {
      return 'Mobile';
    } else if (navigator.userAgent.match(/iPad|Android|Touch/i)) {
      return 'Tablet';
    } else {
      return 'Desktop';
    }
  }

  function send_ros(sel){
    var select = parseInt(sel);
    var d_id = ""+device_id;
    var msg = new ROSLIB.Message({
      header : {
        
      },
      device_id : d_id,
      device_type : "HTTP Request",
      device_model : getPlatformType(),
      selection : select
    });
    selection.publish(msg);
  }

  // Subscribing to a Topic
  // ----------------------

  var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/fsm_status',
    messageType : 'baxter_gbi_core_msgs/status'
  });

  listener.subscribe(function(msg) {
    console.log('Received message on ' + listener.name);
    console.log(msg);
    if(msg.context_type === "menu") {
        renderMenu(msg);
    }

    else if(msg.context_type === "action"){
        renderAction(msg);
    }
  });

var previous_action = "";

function renderMenu(msg){
    document.getElementById("menu_title").innerHTML = msg.m_title;

    document.getElementById("variable_options").innerHTML="";
    document.getElementById("fixed_options").innerHTML="";
    document.getElementById("pbr_msg").innerHTML="";
    document.getElementById("variable_options").style.overflow = 'auto';
    previous_action = "";
    for(let i = 0; i < msg.m_options.length; i++){
        let variable_btn = document.createElement("Button")
        let v_op = document.createTextNode(msg.m_options[i]);
        variable_btn.appendChild(v_op);
        variable_btn.addEventListener('click', function(){
            send_ros(i);
        } );
        document.getElementById("variable_options").appendChild(variable_btn);
        let newline = document.createElement('br')
        document.getElementById("variable_options").appendChild(newline);
    }

    fixedOptions(msg.m_fixed_options, msg.m_options.length);
}

function renderAction(msg){
    // Hide the scroll bar used for the menu
    document.getElementById("variable_options").style.overflow = 'hidden';
    console.log(previous_action)
    console.log(msg.pbr_action === previous_action)
    if (msg.pbr_action === previous_action) {
      document.getElementById("pbr_msg").innerHTML = msg.pbr_msg;
    }
    else{
      document.getElementById("variable_options").innerHTML="";
      document.getElementById("fixed_options").innerHTML="";
      document.getElementById("pbr_msg").innerHTML="";
      previous_action = msg.pbr_action;
      let string = "images/" + msg.pbr_action + ".png";
      document.getElementById("variable_options").innerHTML = "<img alt=figure src="+string+" />";

      document.getElementById("pbr_msg").innerHTML = msg.pbr_msg;

      fixedOptions(msg.action_options, 0);
    }
}

function fixedOptions(f_ops, offset){
    for(let j = 0; j < f_ops.length; j++) {
        let fixed_btn = document.createElement("Button");
        let f_op = document.createTextNode(f_ops[j]);
        fixed_btn.appendChild(f_op);

        fixed_btn.addEventListener('click', function(){
            send_ros(offset + j);
        } );

        // Adjust fixed button width wrt their amount
        let width = 85/f_ops.length;
        fixed_btn.style.width = width + "%";
        document.getElementById("fixed_options").appendChild(fixed_btn);
    }
}