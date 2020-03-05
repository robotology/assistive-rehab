// parameters
// we can run node-button.js specifying --host='..' --listen_port='..' --trigger_port='--'
var folder = process.env.ROBOT_CODE + '/yarp.js/'
var conf = require(folder + 'node_modules/' + 'yargs').argv;
var listen_host;
var listen_port;
var rpc_yarp_port;
if (conf.host) {
    listen_host = conf.host;
}
else {
    listen_host = '192.168.100.193';
}
if (conf.listen_port) {
    listen_port = conf.listen_port;
}
else {
    listen_port = '8080';
}
if (conf.trigger_port) {
    trigger_port = conf.trigger_port;
}
else {
    trigger_port = '/managerTUG/cmd:rpc';
}

// ports
var yarp = require(folder + 'yarp');
var out_yarp_port = new yarp.Port('bottle');
var out_port_name = '/speech/trigger:o' 
out_yarp_port.open(out_port_name);
yarp.Network.connect(out_port_name,trigger_port);
console.log('Connecting to port %s',trigger_port);

// function to collect payload of POST request
const { parse } = require(folder + 'node_modules/' + 'qs');
function collectPayload(request, callback) {
    const FORM_URLENCODED = 'application/x-www-form-urlencoded'; //encoding in the url in the form key1=value1&key2=value2
    if(request.headers['content-type'] === FORM_URLENCODED) {
        let body = '';
        request.on('data', chunk => {
            body += chunk.toString();
        });
        request.on('end', () => {
            callback(parse(body));
        });
    }
    else {
        callback(null);
    }
}

// managing requests
var http = require('http');
var server = http.createServer(function (req, res) {    
    if(req.method == 'POST')
    {
        console.log('Received POST request');
        out_yarp_port.write('trigger');
        console.log('Sending trigger to %s',trigger_port);
        //collectPayload(req, result => {
        //    //console.log(result);
        //    var val = result['key'];
        //    out_yarp_port.write(val);
        //    console.log('Sending command to speech: %s', val);
        //});
    }
    else
    {
        console.log('Undefined request');
    }

});

// run server on listen_host:listen_port
server.listen(listen_port, listen_host);
server.on('listening', function() {
    console.log('Server started on port %s at %s', server.address().port, server.address().address);
});
