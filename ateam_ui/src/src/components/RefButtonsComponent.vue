<template>
    <v-container class="d-flex flex-column">
        <b>Game Controller</b>
        <v-btn
            id="gc_start_button"
            color="green"
            class="d-flex my-1 justify-space-around"
            v-on:click="sendGCCommand('NORMAL_START')">Start</v-btn>
        <v-btn
            id="gc_force_start_button"
            color="green-darken-4"
            class="d-flex my-1 justify-space-around"
            v-on:click="sendGCCommand('FORCE_START')">Force Start</v-btn>
        <v-btn
            id="gc_stop_button"
            color="red"
            class="d-flex my-1 justify-space-around"
            v-on:click="sendGCCommand('STOP')">Stop</v-btn>
        <v-btn
            id="gc_halt_button"
            color="red-darken-4"
            class="d-flex my-1 justify-space-around"
            v-on:click="sendGCCommand('HALT')">Halt</v-btn>
    </v-container>
</template>

<script lang="js">

export default {
    data: function() {
        return {
            connection: null
        }
    },
    created: function() {
        console.log("Opening API websocket connection to GC")
        // TODO(barulicm) The GC IP and port should be parameterized. Not sure how.
        this.connection = new WebSocket("ws://localhost:8081/api/control")
        this.connection.onmessage = function(event) {
            // TODO(barulicm) enable / disable buttons based on game state?
        }
        this.connection.onopen = function(event) {
            console.log("Successfully connected to the GC API server.")
        }
        this.connection.onerror = function(event) {
            console.log(event)
            document.getElementById("gc_start_button").disabled = true;
            document.getElementById("gc_force_start_button").disabled = true;
            document.getElementById("gc_stop_button").disabled = true;
            document.getElementById("gc_halt_button").disabled = true;
        }
    },
    methods: {
        sendGCCommand: function(command_type) {
            console.log("Sending GC command: " + command_type)
            this.connection.send('{ "change": {"origin":"UI", "revertible":true, "newCommand":{"command":{"type":"' + command_type + '", "forTeam":"UNKNOWN"}}}}')
        }
    }
}
</script>
