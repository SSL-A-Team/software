<template>
    <v-container v-if="this.getPlaysWithParams.length == 0">
        <v-col class="d-flex justify-center">
            <v-progress-circular indeterminate class="mx-auto"/>
        </v-col>
    </v-container>
    <div v-else>
        <v-btn block @click.stop="getParamValues" variant="outlined"> Refresh </v-btn>

        <!-- Some of the responsive sizing could be a lot better here -->
        <div v-for="[play_name, play] of this.getPlaysWithParams">

            <v-spacer class="pt-2"/>

            <v-card :title="play.name" variant="outlined">
                <v-container
                    v-for="[param_name, param] of Object.entries(play.params)"
                    class="flex-grow-1 flex-shrink-0"
                >

                    <v-row justify="start" align="center" style="flex-wrap: nowrap;">
                        <v-spacer class="pr-2"/>
                        <div>{{ param_name + ": "}}</div>
                        <v-spacer class="pr-1"/>
                        <input type="text" 
                            v-model="param.commanded_value"
                            class="flex-shrink-1"
                            style="
                                max-width: 4em;
                                background-color: rgb(44, 44, 44);
                            "
                        />
                        <v-spacer class="pr-1"/>
                        <v-btn justify="end" density="compact" @click="updateParam(param)">
                            <v-icon icon="mdi-send" class="mx-0"/>
                        </v-btn>
                    </v-row>
                </v-container>
            </v-card>
        </div>
    </div>

</template>

<script lang="ts">
import { ref, inject } from "vue";
import { Param } from "@/param";
import '@mdi/font/css/materialdesignicons.css'


export default {
    inject: ['state'],
    data() {
        return {
            selectedPlay: []
        }
    },
    mounted() {
        this.getParamValues();

        // TODO: This is bad but roslibjs doesn't seem to expose a way to 
        // wait for updates from the getParams functions
        const self = this;
        setTimeout(function(){ self.getParamValues() }, 3000);
    },
    methods: {
        getParamValues: function() {
            // Update list of params
            this.state.getSTPParams();

            // Get param values from ROS
            for (const [play_name, play] of Object.entries(this.state.plays)) {
                for (const [param_name, param] of Object.entries(play.params)) {
                    param.getValue();
                }
            }
        },
        updateParam(param: Param) {
            param.setValue(param.commanded_value);
        }
    },
    computed: {
        getPlaysWithParams: function() {
            const play_array = Object.entries(this.state.plays)
            if (play_array) {
                return play_array.filter(function(play_object_entry) {
                    const play = play_object_entry[1];
                    return Object.entries(play.params).length;
                });
            }

            return [];
        },
        monitorParamValues: function() {
            // This function is necessary to trigger rerendering when param values
            // change
            let param_values = {};
            for (const [play_name, play] of Object.entries(this.state.plays)) {
                for (const [param_name, param] of Object.entries(play.params)) {
                    param_values[param_name] = param.commanded_value;
                }
            }

            return param_values;
        }
    },
    watch: {
        getPlaysWithParams: {
            handler() {
            },
            deep: true
        },
        monitorParamValues: {
            handler() {
            },
            deep: true
        }
    }
}
</script>
