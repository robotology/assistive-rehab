# How to configure the wifi button

This tutorial will show you how to configure the [Mystrom wifi button](https://mystrom.ch/wifi-button/).

!!! important
    This is required if you want to run the [TUG demo](tug_demo.md) with the speech interaction.

## Using the app

First, you will need to download the [app](https://mystrom.ch/mystrom-app/) and create an account.

Open the app, click on the symbol in the right corner and click on __Add device / WiFi Button__.

Now we will connect the button to your wifi network. Click on __Not connected__ and select __Light__.
Choose the manual configuration.
Go on until this screen appers:

<p align="center"> <img src="https://user-images.githubusercontent.com/9716288/82317034-f8383500-99cd-11ea-9de6-368fc9fd1cb1.jpg" width=350> </p>

Press the button for 1 second and the button will start blinking white.

Go on, this screen will appear:

<p align="center"> <img src="https://user-images.githubusercontent.com/9716288/82317066-01c19d00-99ce-11ea-9692-09c211170581.jpg" width=350> </p>

Keep the button pressed for 2 seconds, the LED will start blinking red and white: now the button will be in Access Point for 5 minutes.

!!! warning "Be patient"
    This might take few seconds.

Go on and select and connect to `my-button-XXXXXX` network, where `XXXXXX` will change depending on the button.

Go back to the app and wait until a list of wifi networks appears.

Choose yours and click on __Use fixed IP__. Now choose an IP address for your button and insert your subnet mask, gateway and DNS.
If required insert the password of your network.

!!! note "How to set the IP address"
    The IP address you choose must be in the same subnet of the machine you will interface to. Thus the first three numbers of the IP address you choose should match those of your machine. Follow [this guide](https://help.edovia.com/hc/en-us/articles/360002865213-Finding-your-computer-s-hostname-or-IP-address) to know the IP address of your machine.

After few seconds the wifi button is found and connected to your network!
Go on and choose a name for it and an icon. Done? Congratulations, now your button configured!!

You can skip the action setting as I will show you how so set actions in next section.

## Setting the actions

[This](https://api.mystrom.ch/?version=latest#36d78676-5ba4-45dc-9eac-f3cb6b4b57fa) is a guide that shows you how to communicate with your device once it has been configured!

To get the device specific information, you can do:

```
curl --location --request GET '[Button IP]/api/v1/device'
```

!!! tip
    Replace `[Button IP]` with the one you chose when configuring the button.

The output will be a `json` object with the mac address (without delimiters) and its field, as shown [here](https://api.mystrom.ch/?version=latest#d973b006-3400-43a6-a653-138dfda2afb9).


Now we want to set the button such that it sends a `POST` request to our IP if pressed once, which corresponds to the `single` action.

!!! Note "Additional actions"
    For the [TUG demo](tug_demo,md), we only use the `single` action. As explained [here](https://api.mystrom.ch/?version=latest#d973b006-3400-43a6-a653-138dfda2afb9), this button also allows you to set `double` (http request executed when pressing the button twice) and `long` (http request executed when pressing the button long).

!!! Question "Not familiar with http requests?"
    Check [this](https://www.tutorialspoint.com/http/http_requests.htm) out!



For doing so, you can do:

```
curl -v -d "single=post://[Your IP]/api/mystrom?key%3Dspeech" http://[Button IP]/api/v1/device/[Button MAC]
```

where:

 - you need to replace `[Your IP]` with the IP address of the machine where you want to receive the `POST` request;
 - you need to replace `[Button IP]` with the IP address of your button;
 - you need to replace `[Button MAC]` with the MAC address of your button;
 - the string `speech` after `key%3D` is the value of the keyword sent with the `POST` request.

!!! tip "Button MAC"
    You can find the MAC address of your button in the app: click on the button, go on __Settings / Technical specifications__.
    Note that in the `curl` command, you need to insert the MAC address with no semicolons.

!!! Failure "Connection refused"
    If you get the following error: __Failed to connect to [Button IP] port 80: No route to host__, it means that the device is into sleep mode.
    This is done in order to preserve battery life (the device is visible in the network only after adding to wifi).

    Luckily, you can enter into maintenance mode, by keeping the button pressed for 5 seconds (it starts blinking green). Now you should be able to communicate again with the device!

In this way, we set the `single` action, which corresponds to the http request executed when pressing the button once.

!!! important
    If you want to use the wifi button to run the [TUG demo](tug_demo.md), configure the `single` action to send a `POST` request to the machine where you run `node-button.js`.

!!! tip
    To set `double` or `long` action, you can follow the same procedure and replace in the `curl` command `single` with the action you want to configure.

Now if you get the device specific information, using the command:

```
curl --location --request GET '[Button IP]/api/v1/device'
```

you will see a `json` output:

```JSON
{
	"[Button MAC]":
    {"type": "button", "battery": true, "reachable": true, "meshroot": false,
      "charge": false, "voltage": 4.179, "fw_version": "2.74.31",
      "single": "post:\/\/[Your IP]]\/api\/mystrom?key=speech",
      "double": "", "long": "", "touch": "", "generic": "",
      "connectionStatus": {"ntp": true, "dns": true, "connection": true,
                          "handshake": true, "login": true}, "name": ""}
}

```

where the `single` action is configured as `"post:\/\/[Your IP]]\/api\/mystrom?key=speech"`.
