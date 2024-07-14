<template>
    <v-container class="d-flex flex-column">
        <b>Field side</b>
        <div
            @mouseover="handleHover(-1)"
            @mouseleave="handleHover(0)"
        >
            <v-switch 
                label="Ignore - Field Side"
                density="compact"
                v-model="negative_side"
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
                v-model="positive_side"
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
            negative_side: false,
            positive_side: false,
        }
    },
    methods: {
        handleSetSide: function(changed_side: number) {
            // Only select one side at a time, deactivate the other side
            if (this.negative_side && this.positive_side) {
                if (changed_side > 0) {
                    this.negative_side = false;
                } else {
                    this.positive_side = false;
                }
            }

            // If neither switch is active set no ignore
            if (!this.negative_side && !this.positive_side) {
                this.state.setIgnoreFieldSide(0);

            // Otherwise the changed switch must be selected now
            } else {
                this.state.setIgnoreFieldSide(changed_side);
            }

        },
        handleHover: function(hovered_side: number) {
            this.state.hovered_field_ignore_side = hovered_side;
        },
    },
    computed: {
        getIgnoreSide: function() {
            return this.state.world.ignore_side;
        }
    },
    watch: {
        getIgnoreSide: {
            handler() {
                if (this.state.world.ignore_side < 0) {
                    this.negative_side = true;
                    this.positive_side = false;
                } else if (this.state.world.ignore_side > 0) {
                    this.negative_side = false;
                    this.positive_side = true;
                } else {
                    this.negative_side = false;
                    this.positive_side = false;
                }
            },
            deep: true
        },
    }
}
</script>
