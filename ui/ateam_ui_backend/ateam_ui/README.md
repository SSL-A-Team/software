# Setting up dependencies
The UI is currently designed to use [Vue](https://vuejs.org/) as a framework to develop the actual framework to lay everything out. Then, [Vite](https://vitejs.dev/) is used to compile everything into efficient contained code. Finally, everything is put into an executable using [nuetralino](https://neutralino.js.org/).

Running the usual `rosdep install` followed by `colcon build` should properly install all dependencies and build the UI.

# Launch files
The `ateam_ui_launch.py` file launches both [rosbridge](https://github.com/RobotWebTools/rosbridge_suite) and the ui.
The `ateam_ui_debug_launch.py` launches rosbridge and runs the ui in development mode where it can be viewed in a web browser.
