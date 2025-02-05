<template>
    <v-container class="d-flex flex-column">
        <b>Game Controller</b>
        <v-container class="d-flex flex-column" v-if="gameControllerCommandsAvailable">
            <v-btn
                id="gc_start_button"
                color="green"
                class="d-flex my-1 justify-space-around"
                v-on:click="sendGCCommand('NORMAL_START', 'UNKNOWN')">Start</v-btn>
            <v-btn
                id="gc_force_start_button"
                color="green-darken-4"
                class="d-flex my-1 justify-space-around"
                v-on:click="sendGCCommand('FORCE_START', 'UNKNOWN')">Force Start</v-btn>
            <v-btn
                id="gc_stop_button"
                color="red"
                class="d-flex my-1 justify-space-around"
                v-on:click="sendGCCommand('STOP', 'UNKNOWN')">Stop</v-btn>
            <v-btn
                id="gc_halt_button"
                color="red-darken-4"
                class="d-flex my-1 justify-space-around"
                v-on:click="sendGCCommand('HALT', 'UNKNOWN')">Halt</v-btn>
            <v-divider :thickness="10"/>
            <v-btn
                id="gc_blue_freekick_button"
                color="blue-darken-3"
                class="d-flex my-1 justify-space-around"
                v-on:click="sendGCCommand('DIRECT', 'BLUE')">Direct</v-btn>
            <v-btn
                id="gc_blue_kickoff_button"
                color="blue-darken-3"
                class="d-flex my-1 justify-space-around"
                v-on:click="sendGCCommand('KICKOFF', 'BLUE')">Kickoff</v-btn>
            <v-btn
                id="gc_blue_penalty_button"
                color="blue-darken-3"
                class="d-flex my-1 justify-space-around"
                v-on:click="sendGCCommand('PENALTY', 'BLUE')">Penalty</v-btn>
            <v-divider :thickness="10"/>
            <v-btn
                id="gc_yellow_freekick_button"
                color="yellow-darken-1"
                class="d-flex my-1 justify-space-around"
                v-on:click="sendGCCommand('DIRECT', 'YELLOW')">Direct</v-btn>
            <v-btn
                id="gc_yellow_kickoff_button"
                color="yellow-darken-1"
                class="d-flex my-1 justify-space-around"
                v-on:click="sendGCCommand('KICKOFF', 'YELLOW')">Kickoff</v-btn>
            <v-btn
                id="gc_yellow_penalty_button"
                color="yellow-darken-1"
                class="d-flex my-1 justify-space-around"
                v-on:click="sendGCCommand('PENALTY', 'YELLOW')">Penalty</v-btn>
            <v-switch 
                label="Auto Continue"
                density="compact"
                v-model="autoContinue"
                @change="toggleContinueAutomatically()"/>
        </v-container>
        <v-container class="d-flex flex-column" v-if="!gameControllerCommandsAvailable">
            <p class="d-flex text-no-wrap my-1 justify-space-around">No local GC.</p>
        </v-container>

        <!-- TODO(barulicm) Add a settings modal for chaning GC address / reconnecting. -->
    </v-container>
</template>

<script lang="js">

export default {
    data: function() {
        return {
            gcAddress: "ws://" + location.hostname + ":8081",
            connection: null,
            connectionState: WebSocket.CLOSED,
            autoContinue: false
        }
    },
    created: function() {
        this.attemptToConnect()
    },
    beforeUnmount: function() {
        this.connection.close()
    },
    methods: {
        sendGCCommand: function(command_type, team) {
            this.connection.send(JSON.stringify({
                change: {
                    origin: "UI",
                    revertible: true,
                    new_command_change: {
                        command: {
                            type: command_type,
                            forTeam: team
                        }
                    }
                }
            }))
        },
        toggleContinueAutomatically: function() {
            this.connection.send(JSON.stringify({
                config_delta: {
                    autoContinue: this.autoContinue
                }
            }))
        },
        attemptToConnect: function() {
            var vue_object = this
            this.connection = new WebSocket(this.gcAddress + "/api/control")
            this.connection.onmessage = function(event) {
                // TODO(barulicm) enable / disable buttons based on game state?
                var output = JSON.parse(event.data)
                if(output.config) {
                    vue_object.autoContinue = output.config.autoContinue
                }
            }
            this.connection.onopen = function(event) {
                console.log("Connected to the GC API server.")
                vue_object.connectionState = event.target.readyState
                // Send empty config delta to get current config
                vue_object.connection.send(JSON.stringify({
                    config_delta: {}
                }))
            }
            this.connection.onerror = function(event) {
                console.log("Error communicating with GC API server:")
                console.log(event)
                vue_object.connectionState = event.target.readyState
            }
            this.connection.onclose = function(event) {
                vue_object.connectionState = event.target.readyState
            }
        }
    },
    computed: {
        gameControllerCommandsAvailable() {
            return this.connectionState === WebSocket.OPEN
        }
    }
}
</script>
