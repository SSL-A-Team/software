<template>
    <v-container v-if="getPlays.size == 0">
        <v-col class="d-flex justify-center">
            <v-progress-circular indeterminate class="mx-auto"/>
        </v-col>
    </v-container>
    <v-col v-else>
        <v-row class="flex-nowrap" justify="space-between">
            <v-btn size="small" @click.stop=setAllPlayEnabledValue(true)> Select All </v-btn>
            <v-btn size="small" @click.stop=setAllPlayEnabledValue(false)> Deselect All </v-btn>
        </v-row>
        <v-row justify="space-between" style="flex-wrap: nowrap;">
            <v-list
                lines="one"
                selectable="true"
                active-color="green"
                v-model:selected="selectedPlay"
                @click:select="selectPlayOverride"
            >
                <v-list-item
                    v-for="[name, play] of getPlays"
                        :key="name"
                        :value="name"
                        :style="{'background-color': getColor(name)}"
                >
                    <v-container class="flex-grow-1 flex-shrink-0 overflow-hidden" style="max-width:11em;">
                        <v-row justify="space-between" style="flex-wrap: nowrap;">
                            <div style="max-width:80%; overflow:hidden; text-wrap:nowrap">
                                {{ name }}
                            </div>
                            <v-spacer class="pr-3"/>
                            <div>{{ displayScore(play.score) }}</div>
                            <v-icon v-if="isScoreInf(play.score)" icon="mdi-infinity" class="mx-0 pl-1 justify-center" size="small"/>
                        </v-row>
                    </v-container>
                </v-list-item>
            </v-list>

            <v-list lines="one" selectable="false" class="flex-grow-0 flex-shrink-1">
                <v-list-item
                    v-for="[name, play] of getPlays"
                        :key="name + 'chkbox'"
                >
                    <input type="checkbox" v-model="play.enabled" v-bind:id="name" @change="setPlayEnabled(play)">
                </v-list-item>
            </v-list>
        </v-row>
    </v-col>

</template>

<script lang="ts">
import { Play } from "@/play";
import "@mdi/font/css/materialdesignicons.css";

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
                this.state.selectedPlayName = selection.id;
            } else {
                this.state.selectedPlayName = "";
            }

            this.state.setOverridePlay(this.state.selectedPlayName);
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
            if (this.state.selectedPlayName == "") {
                this.selectedPlay = [];
            } else {
                this.selectedPlay = [this.state.selectedPlayName];
            }
        },
        setAllPlayEnabledValue(value: boolean) {
            for (var [name, play] of this.state.plays) {
                play.enabled = value;
                this.setPlayEnabled(play);
            }
        },
        getColor(name: string) {
            if (name.toLowerCase().includes("test")) {
                return "#212130FF";
            } else if (name.toLowerCase().includes("pass")) {
                return "#213021FF";
            } else {
                return "#212121FF";
            }
        }
    },
    computed: {
        getPlays: function() {
            return this.state.plays;
        },
        getSelectedPlayName: function() {
            return this.state.selectedPlayName
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
