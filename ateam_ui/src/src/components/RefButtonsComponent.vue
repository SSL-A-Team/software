<template>
    <v-container class="d-flex flex-column">
        <b>Game Controller</b>
        <v-btn
            id="gc_start_button"
            color="green"
            class="d-flex my-1 justify-space-around"
            v-on:click="sendGCCommand('NORMAL_START', 'UNKNOWN')"
            :disabled="!normalStartAllowed">Start</v-btn>
        <v-btn
            id="gc_force_start_button"
            color="green-darken-4"
            class="d-flex my-1 justify-space-around"
            v-on:click="sendGCCommand('FORCE_START', 'UNKNOWN')"
            :disabled="!forceStartAllowed">Force Start</v-btn>
        <v-btn
            id="gc_stop_button"
            color="red"
            class="d-flex my-1 justify-space-around"
            v-on:click="sendGCCommand('STOP', 'UNKNOWN')"
            :disabled="!stopAllowed">Stop</v-btn>
        <v-btn
            id="gc_halt_button"
            color="red-darken-4"
            class="d-flex my-1 justify-space-around"
            v-on:click="sendGCCommand('HALT', 'UNKNOWN')"
            :disabled="!haltAllowed">Halt</v-btn>
        <!-- TODO(barulicm) Add a settings modal for chaning GC address / reconnecting. -->
        <!-- TODO(barulicm) Add buttons / drop down for sending team-specific commands (ie. blue kickoff). -->
    </v-container>
</template>

<script lang="js">

export default {
    data: function() {
        return {
            gcAddress: "ws://localhost:8081",
            connection: null,
            connectionState: WebSocket.CLOSED
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
                    newCommand: {
                        command: {
                            type: command_type,
                            forTeam: team
                        }
                    }
                }
            }))
        },
        attemptToConnect: function() {
            var vue_object = this
            this.connection = new WebSocket(this.gcAddress + "/api/control")
            this.connection.onmessage = function(event) {
                // TODO(barulicm) enable / disable buttons based on game state?
            }
            this.connection.onopen = function(event) {
                console.log("Connected to the GC API server.")
                vue_object.connectionState = event.target.readyState
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
        normalStartAllowed() {
            return this.connectionState === WebSocket.OPEN
        },
        forceStartAllowed() {
            return this.connectionState === WebSocket.OPEN
        },
        stopAllowed() {
            return this.connectionState === WebSocket.OPEN
        },
        haltAllowed() {
            return this.connectionState === WebSocket.OPEN
        },
    }
}
</script>
