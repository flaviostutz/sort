module example

go 1.14

require (
	github.com/flaviostutz/sort v0.0.0-20200302051433-2e941742f1ae
	github.com/konsorten/go-windows-terminal-sequences v1.0.2 // indirect
	github.com/sirupsen/logrus v1.4.2
	golang.org/x/sys v0.0.0-20200302150141-5c8b2ff67527 // indirect
)

replace github.com/flaviostutz/sort => ../

replace github.com/flaviostutz/kalman => ../../kalman
