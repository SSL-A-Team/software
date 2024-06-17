// Types used for params

import ROSLIB from "roslib";

export class Param {
    name: string; // Name of STP parameter (not the full ROS param path)
    ros_param: ROSLIB.Param;
    value: any = null;
    commanded_value: any = null; // holds the input from the param ui component

    constructor(name: string, ros_param: ROSLIB.Param) {
        this.name = name;
        this.ros_param = ros_param;
    }

    getValue() {
        const self = this; // fix dumb javascript things

        this.ros_param.get(
            function(value){
                self.value = value;
                self.commanded_value = value;
            });
    }

    setValue(value: any) {
        const self = this; // fix dumb javascript things
        this.ros_param.set(value, function(response) {
            // For some reason roslibjs refuses to provide
            // response information from parameter service calls
            // if we ever fix it we can reenable this:
            /*
            if (response.success) {
                self.value = value;
                self.commanded_value = value;
            } else {
                console.log("Failed to set ", self.name, ": ", response);
                self.commanded_value = self.value; // if failed reset the ui value
            }
            */

            // For now just assume success
            self.value = value;
            self.commanded_value = value;
        });
    }
}