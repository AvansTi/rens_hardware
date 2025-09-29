# How to

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
 
