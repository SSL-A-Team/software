<template>
    <v-container class="d-flex flex-column">
        <b>Field Side</b>
        <div
            @mouseover="handleHover(-1)"
            @mouseleave="handleHover(0)"
        >
            <v-switch 
                label="Ignore - Field Side"
                density="compact"
                v-model="negativeSide"
                @change="handleSetSide(-1)"
            />
        </div>
        <div
            @mouseover="handleHover(1)"
            @mouseleave="handleHover(0)"
        >
            <v-switch 
                label="Ignore + Field Side"
                density="compact"
                v-model="positiveSide"
                @change="handleSetSide(1)"
            />
        </div>
    </v-container>
</template>

<script lang="ts">

export default {
    inject: ['state'],
    data: function() {
        return {
            negativeSide: false,
            positiveSide: false,
        }
    },
    methods: {
        handleSetSide: function(changedSide: number) {
            // Only select one side at a time, deactivate the other side
            if (this.negativeSide && this.positiveSide) {
                if (changedSide > 0) {
                    this.negativeSide = false;
                } else {
                    this.positiveSide = false;
                }
            }

            // If neither switch is active set no ignore
            if (!this.negativeSide && !this.positiveSide) {
                this.state.setIgnoreFieldSide(0);

            // Otherwise the changed switch must be selected now
            } else {
                this.state.setIgnoreFieldSide(changedSide);
            }

        },
        handleHover: function(hoveredSide: number) {
            this.state.hoveredFieldIgnoreSide = hoveredSide;
        },
    },
    computed: {
        getIgnoreSide: function() {
            return this.state.world.ignoreSide;
        }
    },
    watch: {
        getIgnoreSide: {
            handler() {
                if (this.state.world.ignoreSide < 0) {
                    this.negativeSide = true;
                    this.positiveSide = false;
                } else if (this.state.world.ignoreSide > 0) {
                    this.negativeSide = false;
                    this.positiveSide = true;
                } else {
                    this.negativeSide = false;
                    this.positiveSide = false;
                }
            },
            deep: true
        },
    }
}
</script>
