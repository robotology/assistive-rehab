// parameters
// we can run node-button.js specifying --host=".." --listenPort=".." --triggerPort="--"
/* eslint-disable no-console */
/* eslint-env es6*/
var requireMethod = require;
var folder = process.env.ROBOT_CODE + "/yarp.js/";
var conf = requireMethod(folder + "node_modules/" + "yargs").argv;
var listenHost;
var listenPort;
var triggerPort;
var rpcYarpPort;

if (conf.host) {
    listenHost = conf.host;
}
else {
    listenHost = "192.168.100.193";
}
if (conf.listenPort) {
    listenPort = conf.listenPort;
}
else {
    listenPort = "8080";
}
if (conf.triggerPort) {
    triggerPort = conf.triggerPort;
}
else {
    triggerPort = "/managerTUG/cmd:rpc";
}
// ports
var yarp = requireMethod(folder + "yarp");
var outYarpPort = new yarp.Port("bottle");
outYarpPort.open("/speech/trigger:o");
yarp.Network.connect(outYarpPort.getName(),triggerPort);
console.log("Connecting to port %s",triggerPort);

// function to collect payload of POST request
const { parse } = requireMethod(folder + "node_modules/" + "qs");
function collectPayload(request, callback) {
    const FORM_URLENCODED = "application/x-www-form-urlencoded"; //encoding in the url in the form key1=value1&key2=value2
    if(request.headers["content-type" === FORM_URLENCODED]) {
        let body = "";
        request.on("data", (chunk) => {
            body += chunk.toString();
        });
        request.on("end", () => {
            callback(parse(body));
        });
    }
    else {
        callback(null);
    }
}

// managing requests
var http = requireMethod("http");
var server = http.createServer(function (req, res) {    
    if(req.method === "POST")
    {
        console.log("Received POST request");
        outYarpPort.write("trigger");
        console.log("Sending trigger to %s",triggerPort);
        //collectPayload(req, result => {
        //    //console.log(result);
        //    var val = result["key"];
        //    outYarpPort.write(val);
        //    console.log("Sending command to speech: %s", val);
        //});
    }
    else
    {
        console.log("Undefined request");
    }

});

// run server on listenHost:listenPort
server.listen(listenPort, listenHost);
server.on("listening", function() {
    console.log("Server started on port %s at %s", server.address().port, server.address().address);
});
