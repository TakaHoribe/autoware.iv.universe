var lastTabName = "#main";

$(function() {
    $("#menu").menu();


    // run the currently selected effect
    function runEffect(tabName) {

        if (lastTabName == tabName) {
            return false;
        }

        // most effect types need no options passed by default
        var options = {};
        // run the effect
        $(lastTabName).hide("clip", {
            "direction": "horizontal"
        }, 1000, callback(tabName));
        lastTabName = tabName;
    };

    // callback function to bring a hidden box back
    function callback(tabName) {
        setTimeout(function() {
            $(tabName).removeAttr("style").hide().show("clip", {
                "direction": "horizontal"
            }, 1000);
        }, 1100);
    };

    // set effect from select menu value
    $("#menu_main").click(function() {
        runEffect("#main");
        return false;
    });
    $("#menu_vehicle").click(function() {
        runEffect("#vehicle");
        return false;
    });
});


function ChangeTab(tabName) {
    document.getElementById('main').style.display = 'none';
    document.getElementById('vehicle').style.display = 'none';
    // document.getElementById('status').style.display = 'none';
    document.getElementById(tabName).style.display = 'block';
}

ChangeTab('main');