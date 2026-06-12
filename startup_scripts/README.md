# How to

Copy the 'start_robot.service' to '/etc/systemd/system/':

```
sudo cp ./start_robot.service /etc/systemd/system/
```

Check the status of the service:

```
sudo systemctl status start_robot.service
```

Enable the service:

```
sudo systemctl enable start_robot.service
```

Start the service:

```
sudo systemctl start start_robot.service
```
 
