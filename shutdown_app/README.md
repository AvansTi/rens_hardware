# How to

Make the directory /opt/shutdown-app

```
sudo mkdir /opt/shutdown-app
```

Install the python script into the newly created directory

```
sudo cp shutdown-app.py /opt/shutdown-app/
```

Install the script dependencies

```
sudo apt-get install python3-rpi.gpio
```

Copy the 'shutdown-app.service' to '/lib/systemd/system':

```
sudo cp ./shutdown-app.service /lib/systemd/system/
```

Check the status of the service:

```
sudo systemctl status shutdown-app.service
```

Enable the service:

```
sudo systemctl enable shutdown-app.service
```

Start the service:

```
sudo systemctl start shutdown-app.service
```
 
