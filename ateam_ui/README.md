# Setting up dependencies
The UI is currently designed to use [Vue](https://vuejs.org/) as a framework to develop the actual framework to lay everything out. Then, [Vite](https://vitejs.dev/) is used to compile everything into efficient contained code. Finally, everything is put into an executable using [nuetralino](https://neutralino.js.org/).

1. `cd` to the `ateam_ui/src` folder
2. run `npm i -g @neutralinojs/neu` (I would prefer to use yarn for this so hopefully they can fix their weird issue soon)
3. run `yarn`

Everything should now be ready to be run normally

# Launch files
The `ateam_ui_launch.xml` file should launch both [rosbridge](https://github.com/RobotWebTools/rosbridge_suite) and the ui.

# Notes for development
Do not make changes to files in the `resources` folder, these files are from part of the output from Vite that then gets fed into Neutralino to produce the binaries in the `dist` folder so they will be overwritten every time you build. Most of the files should have their actual counterparts in the `public` folder so make changes there instead.
