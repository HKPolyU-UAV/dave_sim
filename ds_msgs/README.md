# ds_msgs

This is a catch-all package for common vehicle-independent message types.
This package should contain only ROS ```msg``` and ```srv``` files.

All messages contained within should be vehicle-independent and reflect core
vehicle functionality.  Messages for extra cool stuff like fancy navigation
or image interpretation or whatever should live elsewhere.

## Contents of this Package

* Core component messages (control, etc)
* Common data messages
* Messages for ds_diagnostic
* Messages for the JasonTalk wrapper

## Getting Started

This is a DSL ROS package intended to be run as part of the larger DSL ROS ecosystem.
See the [Sentry Wiki](http://sentry-wiki.whoi.edu/ROS_Upgrade)

### Prerequisites

This package stands on its own.  It's a prerequisite for pretty
much everything.

### Installing

You should generally be installing this as part of a rosinstall file.
If handled seperately though, simply add to a catkin workspace and
build with:

```
catkin_make
```

## Deployment

Add to a live system and build.  If the system compiles, it will likely work.
But you should probably run a mission in simulation to make sure.

We use [SemVer](http://semver.org/) for versioning.

## Authors

* **Ian Vaughn** - *Initial work* - [WHOI email](mailto:ivaughn@whoi.edu)
* **Stefano Suman** - *Initial work* - [WHOI email](mailto:ssuman@whoi.edu)

## Acknowledgments

* IFREMER for their architectural support
* Louis Whitcomb et. al. for his message definitions to look over
