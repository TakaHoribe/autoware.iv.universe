if (!Jetson1StatusSubscriber) {
    var Jetson1StatusSubscriber = {
        ros: null,
        name: "",
        init: function() {
            this.ros = new ROSLIB.Ros();
            this.ros.on('error', function(error) {
                document.getElementById('jetson1_info').innerHTML = "Error";
            });
            this.ros.on('connection', function(error) {
                document.getElementById('jetson1_info').innerHTML = "Connected";
            });
            this.ros.on('close', function(error) {
                document.getElementById('jetson1_info').innerHTML = "Close";
            });
            this.ros.connect('ws://' + location.hostname + ':9090');
            var sub = new ROSLIB.Topic({
                ros: this.ros,
                name: '/perception/detection/rois0',
                messageType: 'autoware_perception_msgs/DynamicObjectWithFeatureArray'
            });
            var count = 0;
            setInterval(function(message) {
                const div = document.getElementById("jetson1_status");
                if (div.hasChildNodes()) {
                    div.removeChild(div.firstChild);
                }
                var el = document.createElement("span");
                el.innerHTML = count;
                count = 0;
                document.getElementById("jetson1_status").appendChild(el);
            }, 1000);
            sub.subscribe(function(message) {
                count++;
            });
        }
    }

    Jetson1StatusSubscriber.init();

    window.onload = function() {};
    window.onunload = function() {
        Jetson1StatusSubscriber.ros.close();
    };
}

if (!Jetson2StatusSubscriber) {
    var Jetson2StatusSubscriber = {
        ros: null,
        name: "",
        init: function() {
            this.ros = new ROSLIB.Ros();
            this.ros.on('error', function(error) {
                document.getElementById('jetson2_info').innerHTML = "Error";
            });
            this.ros.on('connection', function(error) {
                document.getElementById('jetson2_info').innerHTML = "Connected";
            });
            this.ros.on('close', function(error) {
                document.getElementById('jetson2_info').innerHTML = "Close";
            });
            this.ros.connect('ws://' + location.hostname + ':9090');
            var sub = new ROSLIB.Topic({
                ros: this.ros,
                name: '/perception/detection/rois1',
                messageType: 'autoware_perception_msgs/DynamicObjectWithFeatureArray'
            });
            var count = 0;
            setInterval(function(message) {
                const div = document.getElementById("jetson2_status");
                if (div.hasChildNodes()) {
                    div.removeChild(div.firstChild);
                }
                var el = document.createElement("span");
                el.innerHTML = count;
                count = 0;
                document.getElementById("jetson2_status").appendChild(el);
            }, 1000);
            sub.subscribe(function(message) {
                count++;
            });
        }
    }


    Jetson2StatusSubscriber.init();

    window.onload = function() {};
    window.onunload = function() {
        Jetson2StatusSubscriber.ros.close();
    };
}

if (!Jetson3StatusSubscriber) {
    var Jetson3StatusSubscriber = {
        ros: null,
        name: "",
        init: function() {
            this.ros = new ROSLIB.Ros();
            this.ros.on('error', function(error) {
                document.getElementById('jetson3_info').innerHTML = "Error";
            });
            this.ros.on('connection', function(error) {
                document.getElementById('jetson3_info').innerHTML = "Connected";
            });
            this.ros.on('close', function(error) {
                document.getElementById('jetson3_info').innerHTML = "Close";
            });
            this.ros.connect('ws://' + location.hostname + ':9090');
            var sub = new ROSLIB.Topic({
                ros: this.ros,
                name: '/perception/detection/rois2',
                messageType: 'autoware_perception_msgs/DynamicObjectWithFeatureArray'
            });
            var count = 0;
            setInterval(function(message) {
                const div = document.getElementById("jetson3_status");
                if (div.hasChildNodes()) {
                    div.removeChild(div.firstChild);
                }
                var el = document.createElement("span");
                el.innerHTML = count;
                count = 0;
                document.getElementById("jetson3_status").appendChild(el);
            }, 1000);
            sub.subscribe(function(message) {
                count++;
            });
        }
    }

    Jetson3StatusSubscriber.init();

    window.onload = function() {};
    window.onunload = function() {
        Jetson3StatusSubscriber.ros.close();
    };
}

if (!Jetson4StatusSubscriber) {
    var Jetson4StatusSubscriber = {
        ros: null,
        name: "",
        init: function() {
            this.ros = new ROSLIB.Ros();
            this.ros.on('error', function(error) {
                document.getElementById('jetson4_info').innerHTML = "Error";
            });
            this.ros.on('connection', function(error) {
                document.getElementById('jetson4_info').innerHTML = "Connected";
            });
            this.ros.on('close', function(error) {
                document.getElementById('jetson4_info').innerHTML = "Close";
            });
            this.ros.connect('ws://' + location.hostname + ':9090');
            var sub = new ROSLIB.Topic({
                ros: this.ros,
                name: '/perception/detection/rois3',
                messageType: 'autoware_perception_msgs/DynamicObjectWithFeatureArray'
            });
            var count = 0;
            setInterval(function(message) {
                const div = document.getElementById("jetson4_status");
                if (div.hasChildNodes()) {
                    div.removeChild(div.firstChild);
                }
                var el = document.createElement("span");
                el.innerHTML = count;
                count = 0;
                document.getElementById("jetson4_status").appendChild(el);
            }, 1000);
            sub.subscribe(function(message) {
                count++;
            });
        }
    }

    Jetson4StatusSubscriber.init();

    window.onload = function() {};
    window.onunload = function() {
        Jetson4StatusSubscriber.ros.close();
    };
}
if (!Jetson5StatusSubscriber) {
    var Jetson5StatusSubscriber = {
        ros: null,
        name: "",
        init: function() {
            this.ros = new ROSLIB.Ros();
            this.ros.on('error', function(error) {
                document.getElementById('jetson5_info').innerHTML = "Error";
            });
            this.ros.on('connection', function(error) {
                document.getElementById('jetson5_info').innerHTML = "Connected";
            });
            this.ros.on('close', function(error) {
                document.getElementById('jetson5_info').innerHTML = "Close";
            });
            this.ros.connect('ws://' + location.hostname + ':9090');
            var sub = new ROSLIB.Topic({
                ros: this.ros,
                name: '/perception/detection/rois4',
                messageType: 'autoware_perception_msgs/DynamicObjectWithFeatureArray'
            });

            var count = 0;
            setInterval(function(message) {
                const div = document.getElementById("jetson5_status");
                if (div.hasChildNodes()) {
                    div.removeChild(div.firstChild);
                }
                var el = document.createElement("span");
                el.innerHTML = count;
                count = 0;
                document.getElementById("jetson5_status").appendChild(el);
            }, 1000);
            sub.subscribe(function(message) {
                count++;
            });
        }
    }
    Jetson5StatusSubscriber.init();

    window.onload = function() {};
    window.onunload = function() {
        Jetson5StatusSubscriber.ros.close();
    };
}

if (!Jetson6StatusSubscriber) {
    var Jetson6StatusSubscriber = {
        ros: null,
        name: "",
        init: function() {
            this.ros = new ROSLIB.Ros();
            this.ros.on('error', function(error) {
                document.getElementById('jetson6_info').innerHTML = "Error";
            });
            this.ros.on('connection', function(error) {
                document.getElementById('jetson6_info').innerHTML = "Connected";
            });
            this.ros.on('close', function(error) {
                document.getElementById('jetson6_info').innerHTML = "Close";
            });
            this.ros.connect('ws://' + location.hostname + ':9090');
            var sub = new ROSLIB.Topic({
                ros: this.ros,
                name: '/perception/detection/rois5',
                messageType: 'autoware_perception_msgs/DynamicObjectWithFeatureArray'
            });
            var count = 0;
            setInterval(function(message) {
                const div = document.getElementById("jetson6_status");
                if (div.hasChildNodes()) {
                    div.removeChild(div.firstChild);
                }
                var el = document.createElement("span");
                el.innerHTML = count;
                count = 0;
                document.getElementById("jetson6_status").appendChild(el);
            }, 1000);
            sub.subscribe(function(message) {
                count++;
            });
        }
    }

    Jetson6StatusSubscriber.init();

    window.onload = function() {};
    window.onunload = function() {
        Jetson6StatusSubscriber.ros.close();
    };
}