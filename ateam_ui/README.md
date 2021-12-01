# Setting up dependencies
The UI is currently designed to use [Vue](https://vuejs.org/) as a framework to develop the actual framework to lay everything out. Then, [Vite](https://vitejs.dev/) is used to compile everything into efficient contained code. Finally, everything is put into an executable using [nuetralino](https://neutralino.js.org/).

1. `cd` to the `ateam_ui/src` folder
2. run `npm i -g @neutralinojs/neu` (I would prefer to use yarn for this so hopefully they can fix their weird issue soon)
3. run `yarn`

Everything should now be ready to be run normally

# Launch files
The `ateam_ui_launch.xml` file should launch both [rosbridge](https://github.com/RobotWebTools/rosbridge_suite) and the ui.
The `ateam_ui_debug_launch.xml` launches both rosbridge and runs the ui in development mode where it can be viewed in a web browser.

# Notes for development
When running `colcon build` the `/src` directory is copied into the share folder of the package where everything can then be built using the launch files discussed above. Eventually I should probably change it so colcon build actually builds the javascript packages and the launch files just run them but this works well enough for now.
