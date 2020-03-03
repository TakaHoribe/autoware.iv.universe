if (!GoHomePublisher) {
    var GoHomePublisher = {
        ros: null,
        name: "",
        init: function() {
            this.ros = new ROSLIB.Ros();
            this.ros.on('error', function(error) {
                document.getElementById('go_home_info').innerHTML = "Error";
            });
            this.ros.on('connection', function(error) {
                document.getElementById('go_home_info').innerHTML = "Connected";
            });
            this.ros.on('close', function(error) {
                document.getElementById('go_home_info').innerHTML = "Closed";
            });
            this.ros.connect('ws://' + location.hostname + ':9090');
        },
        send: function() {
            var pub = new ROSLIB.Topic({
                ros: this.ros,
                name: '/move_base_simple/goal',
                messageType: 'geometry_msgs/PoseStamped'
            });
            var str = new ROSLIB.Message({
                header: {
                    seq: 0,
                    stamp: {
                        secs: 0,
                        nsecs: 0
                    },
                    frame_id: "viewer"
                },
                pose: {
                    position: {
                        x: 64.5089035034,
                        y: 31.2445583344,
                        z: 0.0
                    },
                    orientation: {
                        x: 0.0,
                        y: 0.0,
                        z: -0.715335465306,
                        w: 0.698781204724
                    }
                }
            });
            pub.publish(str);
        }
    }
    GoHomePublisher.init();

    window.onload = function() {};
    window.onunload = function() {
        GoHomePublisher.ros.close();
    };
}