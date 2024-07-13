<template>
    <canvas ref="canvas" style="width:100%; height:auto; display:block;" />
</template>


<script lang="ts">
import { shallowRef, ref, inject } from 'vue';
import * as PIXI from 'pixi.js';

export default {
    inject: ['state'],
    data() {
        return {
            pixi: {}
        }
    },
    mounted() {
        // size is based on 140 pixels per meter times the standard max field size including boundaries (13.4 x 10.4m)
        // TODO: Make this properly resize itself
        const width = 1876;
        const height = 1456;
        let pixi = new PIXI.Application({
            width: width,
            height: height,
            background: 'green',
            antialias: true,
            view: this.$refs.canvas
        });
        this.pixi = shallowRef(pixi);
        this.state.world.field.initializePixi(this.pixi, this.state);

        // TODO: clean this up
        this.pixi.stage.eventMode = 'static';

    },
    methods: {
        update: function() {
            this.state.world.field.update(this.pixi, this.state);
        },
        onClick: function(event: UIEvent) {
            // TODO: Replace this with ball/robot drag code. Currently having an issue where
            // enabling event handling on field elements throws an error about not having
            // a propagation path
        }
    },
    computed: {
        getFieldDimensions: function() {
            return this.state.world.field.fieldDimensions;
        }
    },
    watch: {
        getFieldDimensions: {
            handler() {
                this.state.world.field.drawFieldLines(this.pixi.stage.getChildByName("fieldLines"), this.state);
            },
            deep: true
        }
    }
}
</script>
