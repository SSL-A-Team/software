<template>
    <v-menu
        activator="parent"
        :open-on-hover="false"
        :open-on-click="false"
        :model-value="activeDetailedMenuId==robot.id"
        :close-on-content-click="false"
        location-strategy="connected"
        location="start center"
        origin="overlap"
        :offset-x="200"
        :offset-y="200"
        scroll-strategy="reposition"
    >
        <v-row>
            <v-btn :ref="'restart' + robot.id" dense class="mx-1" style="max-width: 50;"
                @mousedown="startHold('restart', robot)" 
                @mouseup="stopHold('restart', robot)"
                @mouseleave="stopHold('restart', robot)"
            >
                <v-icon icon="mdi-restart"/>
            </v-btn>
            <v-btn :ref="'shutdown' + robot.id" dense class="mx-1" style="max-width: 50;"
                @mousedown="startHold('shutdown', robot)" 
                @mouseup="stopHold('shutdown', robot)"
                @mouseleave="stopHold('shutdown', robot)"
            >
                <v-icon icon="mdi-power"/>
            </v-btn>

        </v-row>
    </v-menu>
</template>

<script lang="ts">
import { AppState } from "@/state";
import { Robot } from "@/robot";
import { inject } from "vue";

export default {
    inject: ['state'],
    props: {
        robot: {
            type: Robot,
            required: true
        },
        activeDetailedMenuId: {
            type: Number,
            required: true
        }
    },
    data() {
        return {
            state: inject('state') as AppState,
            holdStartTime: null,
            timeoutId: null
        }
    },
    methods: {
        startHold: function(type: string, robot: Robot) {
            if (this.holdStartTime === null) {
                this.holdStartTime = performance.now();
            }

            if (this.timeoutId){
                clearTimeout(this.timeoutId);
            }

            const object = this;
            this.timeoutId = setTimeout(function() {
                object.holdStartTime = null;
                object.timeoutId = null;
                // object.activeDetailedMenuId = null;

                object.state.sendPowerRequest(robot.id, type);
            }, 1000);
        },
        stopHold: function(type: string, robot: Robot) {
            this.holdStartTime = null;
            if (this.timeoutId){
                clearTimeout(this.timeoutId);
            }
        },
    }
}
</script>
