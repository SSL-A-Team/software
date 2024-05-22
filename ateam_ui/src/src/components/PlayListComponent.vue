<template>
    <v-container v-if="!getPlays">
        <v-col class="d-flex justify-center">
            <v-progress-circular indeterminate class="mx-auto"/>
        </v-col>
    </v-container>
    <v-else>
        <v-row justify="space-between" style="flex-wrap: nowrap;">
            <v-list 
                lines="one"
                selectable="true"
                active-color="green"
                @click:select="selectPlayOverride"
            >
                <v-list-item
                    v-for="play of this.getPlays"
                        :key="play.name"
                        :value="play"
                >
                    {{ play.name }}
                </v-list-item>
            </v-list>

            <v-list lines="one" selectable="false">
                <v-list-item
                    v-for="play of this.getPlays"
                        :key="play.name + 'chkbox'"
                >
                    <input type="checkbox" v-model="play.enabled" v-bind:id="play.name" @change="setPlayEnabled(play)">
                </v-list-item>
            </v-list>
        </v-row>
    </v-else>

</template>

<script lang="ts">
import { ref, inject } from "vue";
import { AIState } from "@/AI";
import { Referee, GameStage, GameCommand} from "@/referee";
import AIRecursiveComponent from "./AIRecursiveComponent.vue";

export default {
    inject: ['state'],
    mounted() {
        this.state.getPlayNames()
    },
    methods: {
        selectPlayOverride(selection) {
            let play_name = "";
            if (selection.value) {
                this.state.selected_play = selection.id;
                play_name = this.state.selected_play.name;
            } else {
                this.state.selected_play = null
            }

            this.state.setOverridePlay(play_name);
        },
        setPlayEnabled(play) {
            this.state.setPlayEnabled(play)
        }
    },
    computed: {
        getPlays: function() {
            return this.state.plays;
        },
        getSelectedPlayName: function() {
            if (this.state.selected_play != null) {
                return this.state.selected_play.name
            }

            return "N/A"
        }
    },
    watch: {
        getPlays: {
            handler() {
            },
            deep: true
        },
        getSelectedPlayName: {
            handler() {
            },
            deep: true
        }
    }
}
</script>
