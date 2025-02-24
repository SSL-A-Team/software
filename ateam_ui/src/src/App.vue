<template>
    <v-app>
        <v-app-bar app ref="Top Bar" color="#cbb26b" density="compact">
            <v-app-bar-title> ATeam UI </v-app-bar-title>
        </v-app-bar>
        <v-main>
            <v-container fluid class="d-inline-flex justify-space-between">
            <v-row class="flex-nowrap">
                <v-col class="flex-grow-0 flex-shrink-0">
                    <GoaliePickerComponent/>
                    <FieldSideComponent/>
                    <RefButtonsComponent v-if="!state.comp" />
                </v-col>
                <v-col class="flex-grow-0 flex-shrink-0">
                    <StatusComponent ref="robotStatus"/>
                </v-col>
                <v-col class="flex-grow-1 flex-shrink-1" style="height: auto">
                    <HistoryComponent ref="historyComponent"/>
                    <GameStatusComponent ref="refStatus"/>
                    <FieldComponent ref="mainField" class="ma-2 pa-2"/>
                </v-col>
                <v-col class="flex-grow-0 flex-shrink-0 justify-center" style="max-width:15vw; min-width:17em">
                    <v-card>
                        <v-tabs v-model="tab" show-arrows="false">
                            <v-tab value="data_tree">Data Tree</v-tab>
                            <v-tab value="play_book">Play List</v-tab>
                        </v-tabs>

                        <v-card-text>
                            <v-window v-model="tab">
                                <v-window-item value="data_tree">
                                    <AIComponent ref="AIStatus"/>
                                </v-window-item>

                                <v-window-item value="play_book">
                                    <PlaybookComponent ref="Play Book"/>
                                </v-window-item>
                            </v-window>
                        </v-card-text>
                    </v-card>
                </v-col>
            </v-row>
            </v-container>
        </v-main>
    </v-app>
</template>


<script lang="ts">
import { AppState } from "@/state";

import FieldComponent from './components/FieldComponent.vue'
import StatusComponent from './components/StatusComponent.vue'
import RefButtonsComponent from './components/RefButtonsComponent.vue'
import GameStatusComponent from './components/GameStatusComponent.vue'
import AIComponent from './components/AIComponent.vue'
import PlaybookComponent from './components/PlaybookComponent.vue'
import FieldSideComponent from './components/FieldSideComponent.vue'
import GoaliePickerComponent from './components/GoaliePickerComponent.vue'
import HistoryComponent from './components/HistoryComponent.vue'

export default {
    data() {
        return {
            intervalIds: [] as NodeJS.Timer[],
            state: new AppState(),
            renderConfig: {
                angle: 0,
                scale: 75, // Pixels per meter (in the rendering canvas)
                factor: 1 // Field Scaling Factor (applied to the canvas when it's added to the UI)
            },
            tab: null
        }
    },
    provide() { 
        return {
            state: this.state,
            renderConfig: this.renderConfig
        }
    },
    methods: {
        // Renders field at 100fps
        updateField: function() {

            // Hack for testing, need to find a better way to trigger update when kenobi is not available
            const timestamp = Date.now();
            this.state.realtimeWorld.timestamp = timestamp;

            // Only store history while we are unpausued
            if (this.state.selectedHistoryFrame == -1) {
                // Only render while unpaused, HistoryComponent will handle field rendering while not in realtime
                this.$refs.mainField.update();

                this.state.historyEndIndex++;
                if (this.state.worldHistory.length < 100000) {
                    this.state.worldHistory.push(structuredClone(this.state.world.__v_raw));
                } else {
                    if (this.state.historyEndIndex >= this.state.worldHistory.length) {
                        this.state.historyEndIndex = 0;
                    }
                    this.state.worldHistory[this.state.historyEndIndex] = structuredClone(this.state.world.__v_raw);
                }
            }
        },
        updateStatus: function() {
            this.$refs.robotStatus.update();
        }
    },
    beforeUnmount() {
        for (const interval of this.intervalIds) {
            clearInterval(interval);
        }
    },
    created() {
        this.intervalIds.push(setInterval(this.updateField, 10));
        this.intervalIds.push(setInterval(this.updateStatus, 100));
    },
    mounted() {
        // This has to be called after Vue has started monitoring the properties so that the callbacks
        // get registered to track for updates
        this.state.connectToRos();
    },
    components: {
        FieldComponent,
        StatusComponent,
        RefButtonsComponent,
        GameStatusComponent,
        AIComponent,
        PlaybookComponent,
        FieldSideComponent,
        GoaliePickerComponent,
        HistoryComponent
    }
}
</script>
