<template>
    <v-app>
        <v-app-bar app ref="Top Bar" color="ateam-color" density="compact">
            <v-app-bar-title class="flex-0-0" v-text="'Ateam UI'" style="text-wrap-mode:nowrap;"/>
            <MenuBarComponent/>
        </v-app-bar>
        <v-main>
            <v-container fluid class="d-inline-flex justify-space-between">
            <v-row class>
                <v-col class="flex-grow-0 flex-shrink-0" style="min-width:12vw">
                    <GoaliePickerComponent/>
                    <FieldSideComponent/>
                    <RefButtonsComponent v-if="!state.comp" />
                </v-col>
                <StatusComponent ref="robotStatus"/>
                <v-col class="flex-grow-1 flex-shrink-1" style="height: auto">
                    <HistoryComponent ref="historyComponent"/>
                    <GameStatusComponent ref="refStatus"/>
                    <FieldComponent ref="mainField" class="ma-2 pa-2"/>
                </v-col>
                <v-col class="flex-grow-0 flex-shrink-0 justify-center overflow-y-auto" style="height: 100vh; max-width:18vw; min-width:18vw">
                    <v-card>
                        <v-tabs v-model="tab" show-arrows="false">
                            <v-tab value="dataTree">Data Tree</v-tab>
                            <v-tab value="playBook">Play List</v-tab>
                        </v-tabs>

                        <v-card-text>
                            <v-window v-model="tab">
                                <v-window-item value="dataTree">
                                    <AIComponent ref="AIStatus"/>
                                </v-window-item>

                                <v-window-item value="playBook">
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

import FieldComponent from "./components/FieldComponent.vue";
import StatusComponent from "./components/StatusComponent.vue";
import RefButtonsComponent from "./components/RefButtonsComponent.vue";
import GameStatusComponent from "./components/GameStatusComponent.vue";
import AIComponent from "./components/AIComponent.vue";
import PlaybookComponent from "./components/PlaybookComponent.vue";
import FieldSideComponent from "./components/FieldSideComponent.vue";
import GoaliePickerComponent from "./components/GoaliePickerComponent.vue";
import HistoryComponent from "./components/HistoryComponent.vue";
import MenuBarComponent from "./components/MenuBarComponent.vue";

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

            if (!this.state.useKenobi) {
                this.state.realtimeWorld.timestamp = Date.now();
                this.state.updateHistory()
            }

            // Only render while unpaused, HistoryComponent will handle field rendering while not in realtime
            if (this.state.selectedHistoryFrame == -1) {
                this.$refs.mainField.update();
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
        HistoryComponent,
        MenuBarComponent
    }
}
</script>
