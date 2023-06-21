<template>
    <v-app>
        <v-app-bar app ref="Top Bar" color="#cbb26b" density="compact">
            <v-app-bar-title> ATeam UI </v-app-bar-title>
        </v-app-bar>
        <v-main>
            <v-container class="d-flex flex-row" ref="mainComponents">
                <RefButtonsComponent/>
                <StatusComponent/>
                <FieldComponent ref="mainField"/>
            </v-container>
        </v-main>
    </v-app>
</template>


<script lang="ts">

import FieldComponent from './components/FieldComponent.vue'
import StatusComponent from './components/StatusComponent.vue'
import RefButtonsComponent from './components/RefButtonsComponent.vue'
import { provide } from 'vue'
import { defineComponent, toRaw } from 'vue'

import { AppState } from '@/state'


export default {
    data() {
        return {
            intervalId: null,
            state: new AppState(),
            renderConfig: {
                angle: 0,
                scale: 75, // Pixels per meter (in the rendering canvas)
                factor: 1 // Field Scaling Factor (applied to the canvas when it's added to the UI)
            }
        }
    },
    provide() { 
        return {
            state: this.state,
            renderConfig: this.renderConfig
        }
    },
    methods: {
        // Stores History and renders field at 100fps
        update: function() {
            // This should store a bit over 15 minutes of history, we can bump it higher
            // if the memory usage isn't too excessive (I think this should only be ~50MB)
            if (this.state.history.length < 100000) {
                // if this doesn't properly deep copy we may need Lodash
                this.state.history.push(structuredClone(toRaw(this.state.world)));
            }

            // update components
            this.$refs.mainField.update();
        }
    },
    beforeUnmount() {
        clearInterval(this.intervalId);
    },
    created() {
        this.intervalId = setInterval(this.update, 1);
    },
    components: {
        FieldComponent,
        StatusComponent,
        RefButtonsComponent
    }
}
</script>
