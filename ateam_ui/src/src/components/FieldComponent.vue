<template>
    <canvas ref="canvas"/>
</template>


<script lang="ts">
import { ref, inject } from 'vue';
import * as PIXI from 'pixi.js';

export default {
    inject: ['state', 'renderConfig'],
    data() {
        return {
            pixi: {}
        }
    },
    mounted() {
        // doesn't seem to match the size given
        this.pixi = new PIXI.Application({
            width: 600,
            height: 450,
            background: 'green',
            view: this.$refs.canvas
        });

        this.state.world.field.initializePixi(this.pixi);
    },
    methods: {
        update: function() {
            this.state.world.field.update(this.pixi);
        },
    }
}
</script>
