<template>
    <v-app>
        <v-app-bar app ref="Top Bar" color="#cbb26b" density="compact">
            <v-app-bar-title> ATeam UI </v-app-bar-title>
        </v-app-bar>
        <v-main>
            <v-container fluid class="d-inline-flex justify-space-between">
            <v-row class="flex-nowrap">
                <v-col v-if="!this.state.comp" class="flex-grow-0 flex-shrink-0">
                    <RefButtonsComponent/>
                    <FieldHalfComponent/>
                </v-col>
                <v-col class="flex-grow-0 flex-shrink-0">
                    <StatusComponent ref="robotStatus"/>
                </v-col>
                <v-col class="flex-grow-1 flex-shrink-1" style="height: auto">
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

import FieldComponent from './components/FieldComponent.vue'
import StatusComponent from './components/StatusComponent.vue'
import RefButtonsComponent from './components/RefButtonsComponent.vue'
import GameStatusComponent from './components/GameStatusComponent.vue'
import AIComponent from './components/AIComponent.vue'
import PlaybookComponent from './components/PlaybookComponent.vue'
import FieldHalfComponent from './components/FieldHalfComponent.vue'
import { provide } from 'vue'
import { defineComponent, toRaw } from 'vue'

import { AppState } from '@/state'


export default {
    data() {
        return {
            intervalIds:  [],
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
            this.$refs.mainField.update();

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
        this.state.mount();
    },
    components: {
        FieldComponent,
        StatusComponent,
        RefButtonsComponent,
        GameStatusComponent,
        AIComponent,
        PlaybookComponent,
        FieldHalfComponent
    }
}
</script>
