<template>
    <v-app>
        <v-app-bar color="red" density="compact">
            <v-app-bar-title> ATeam UI </v-app-bar-title>
        </v-app-bar>
        <v-main>
            <FieldComponent/>
        </v-main>
    </v-app>
</template>


<!-- <template>
     <v-app>
     <v-app-bar color="red" density="compact">
     <v-app-bar-title> ATeam UI </v-app-bar-title>
     </v-app-bar>
     <v-main>
     <canvas ref="canvas" width="560" height="360" style="background: green;">
     <FieldComponent/>
     </canvas>
     </v-main>
     </v-app>
     </template> -->


<script lang="js">
import FieldComponent from './components/FieldComponent.vue'
import { defineComponent, provide, reactive, ref, computed } from 'vue'

const robots = [];
var yoffset = 0;
for (var color of ["yellow", "blue"]) {
    var xoffset = 0;
    for (var i = 0; i < 16; i++) {
        robots.push({
            id: i,
            team: color,
            x: 60 + (70*(i+xoffset)),
            y: 60 + (70*yoffset)
        })

        if (i!=0 && ((i+1) % 8) == 0) {
            xoffset-=8;
            yoffset+=1;
        }
    };
    yoffset += 1;
}

export default {
    data() {
        return {
            state: {
                robots: robots,
                sim: true
            },
            // field: {
            //     canvas: null,
            //     ctx: null
            // }
        }
    },
    provide() {
        return {
            state: this.state,
        }
    },
    mounted() {
        // this.field.canvas = this.$refs['canvas'];
        // this.field.ctx = this.field.canvas.getContext('2d');
    },
    methods: {
    },
    components: {FieldComponent}
}

</script>
