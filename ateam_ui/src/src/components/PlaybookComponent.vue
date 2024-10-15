<template>
    <v-container v-if="Object.entries(this.getPlays).length == 0">
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
                v-model:selected="selectedPlay"
                @click:select="selectPlayOverride"
            >
                <v-list-item
                    v-for="[name, play] of Object.entries(this.getPlays)"
                        :key="name"
                        :value="name"
                >
                    <v-container class="flex-grow-1 flex-shrink-0 overflow-hidden" style="max-width:11em;">
                        <v-row  justify="space-between" style="flex-wrap: nowrap;">
                            <div style="max-width:80%; overflow:hidden; text-wrap:nowrap">
                                {{ name }}
                            </div>
                            <v-spacer class="pr-3"/>
                            <div>{{ this.displayScore(play.score) }}</div>
                            <v-icon v-if="this.isScoreInf(play.score)" icon="mdi-infinity" class="mx-0 pl-1 justify-center" size="small"/>
                        </v-row>
                    </v-container>
                </v-list-item>
            </v-list>

            <v-list lines="one" selectable="false" class="flex-grow-0 flex-shrink-1">
                <v-list-item
                    v-for="[name, play] of Object.entries(this.getPlays)"
                        :key="name + 'chkbox'"
                >
                    <input type="checkbox" v-model="play.enabled" v-bind:id="name" @change="setPlayEnabled(play)">
                </v-list-item>
            </v-list>
        </v-row>
    </v-else>

</template>

<script lang="ts">
import { ref, inject } from "vue";
import { AIState } from "@/AI";
import { Play } from "@/play";
import { Referee, GameStage, GameCommand} from "@/referee";
import AIRecursiveComponent from "./AIRecursiveComponent.vue";
import '@mdi/font/css/materialdesignicons.css'

export default {
    inject: ['state'],
    data() {
        return {
            selectedPlay: []
        }
    },
    mounted() {
        this.setListSelectedPlay();
    },
    methods: {
        selectPlayOverride(selection) {
            if (selection.value) {
                this.state.selected_play_name = selection.id;
            } else {
                this.state.selected_play_name = "";
            }

            this.state.setOverridePlay(this.state.selected_play_name);
        },
        setPlayEnabled(play: Play) {
            this.state.setPlayEnabled(play)
        },
        displayScore(score: number) {
            if (score == null) {
                return "NaN";
            } else if (score == Number.MAX_VALUE) {
                // Infinity symbol
                return "";
            } else if (score == -Number.MAX_VALUE) {
                // Negative Infinity symbol
                return "-";
            } else {
                // Round to 2 decimal places
                return score.toFixed(2);
            }
        },
        isScoreInf(score: number){
            if (score == Number.MAX_VALUE || score == -Number.MAX_VALUE) {
                return true
            } else {
                return false
            }
        },
        setListSelectedPlay() {
            if (this.state.selected_play_name == "") {
                this.selectedPlay = [];
            } else {
                this.selectedPlay = [this.state.selected_play_name];
            }
        }
    },
    computed: {
        getPlays: function() {
            return this.state.plays;
        },
        getSelectedPlayName: function() {
            return this.state.selected_play_name
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
                this.setListSelectedPlay();
            },
            deep: true
        }
    }
}
</script>
