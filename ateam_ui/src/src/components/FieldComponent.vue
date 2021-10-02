<template>
    <v-stage ref="stage" :config="configKonva" @dragstart="handleDragStart" @dragend="handleDragEnd">
    </v-stage>
</template>


<script lang="js">
import { ref, defineComponent, inject, provide, reactive, watchEffect } from 'vue';

export default {
    inject: ['state'],
    data() {
        return {
            field: null,
            overLayer: null,
            robotLayer: null,
            robots: [],
            configKonva: {
                width: 560,
                height: 360
            }
        }
    },
    mounted() {
        this.stage = this.$refs["stage"].getStage();
        this.overlay = new Konva.Layer();
        this.field = new Konva.Layer();

        this.stage.add(this.overlay);
        this.stage.add(this.field);

        const background = new Konva.Rect({
            x:0,
            y:0,
            width: this.field.width(),
            height: this.field.height(),
            fill: "green"
        })
        this.field.add(background);


        // robot shapes not reacting to changes in state
        // Outline on text is missing in neutralino

        // for (var robot of this.state.robots) {
        for (var i = 0; i < 32; i++) {
            const robot = this.state.robots[i];
            const bot = new Konva.Shape({
                sceneFunc: function(ctx) {

                    const scale = 30; //pixels per meter
                    const radius = .9;
                    const sr = scale*radius;

                    const start = (-50/180)*Math.PI;
                    const end =  (230/180)*Math.PI;

                    ctx.beginPath();
                    ctx.arc(-sr, -sr, sr, start, end);
                    ctx.closePath();
                    ctx.fillStrokeShape(this);


                    ctx.strokeStyle = "black"; //this will need to support styles later
                    ctx.fillStyle = "white";
                    ctx.textAlign = "center";
                    ctx.textBaseline = "middle"
                    ctx.font = "30px sans-serif";
                    ctx.strokeText(this.getAttr('r_id'), -sr, -sr + 4);
                    ctx.fillText(this.getAttr('r_id'), -sr, -sr + 4);
                },
                r_id: robot.id,
                team: robot.team,
                x: robot.x,
                y: this.state.robots[i].y,
                fill: robot.team,
                draggable: this.state.sim,
                stroke: 'black',
                strokeWidth: 2

            });
            this.robots.push(bot);
            this.field.add(bot);

        }
        watchEffect(()=>console.log(this.state.robots[0].y))

        this.stage.on('click', function(e){
            var pos = this.getRelativePointerPosition();
            this.children[1].add(new Konva.Rect({
                x: pos.x,
                y: pos.y,
                width: 20,
                height: 20,
                fill: "red"
            }))
        })

    },
    methods: {
        handleDragStart: function(e) {
            e.target.moveToTop();
            console.log(this.state.robots[0]);
        },
        handleDragEnd: function(e) {
            console.log("drag end");
        }
    },
    render(){
        console.log("Field Render");
    }
}



// export default {
//     inject: ['state', 'field'],
//     data() {
//         return{ templates: {} }
//     },
//     mounted(){
//         console.log("mounted field")
//         this.templates = this.generateTemplates(["yellow", "blue"]);
//     },
//     render(){
//         console.log("rendering field");
//         if (!this.field.ctx) return;
//         const ctx = this.field.ctx;
//
//         ctx.clearRect(0,0,800,800);
//
//         for (var robot of this.state.robots) {
//             this.drawRobot(robot);
//         }
//
//         // ctx.drawImage(this.templates.yellow, 200, 200);
//         // ctx.drawImage(this.templates.blue, 200, 150);
//     },
//     methods: {
//         generateTemplates: function(colors) {
//             var templates = {}
//             for (var color of colors) {
//                 var c = document.createElement('canvas');
//                 c.width = 20;
//                 c.height = 20;
//
//                 var tc = c.getContext('2d');
//                 tc.fillStyle = color; //this will need to support styles later
//                 tc.fillRect(0,0,20,20);
//
//                 templates[color] = c;
//             }
//             return templates;
//         },
//         drawRobot: function(robot) {
//             this.field.ctx.drawImage(this.templates[robot.team], robot.x - 10, robot.y - 10);
//             this.field.ctx.strokeStyle = "black"; //this will need to support styles later
//             this.field.ctx.fillStyle = "white";
//             this.field.ctx.textAlign = "center";
//             this.field.ctx.textBaseline = "middle"
//             this.field.ctx.strokeText(robot.id, robot.x, robot.y);
//             this.field.ctx.fillText(robot.id, robot.x, robot.y);
//
//
//         }
//     }
// }
</script>
