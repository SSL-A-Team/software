<template>
    <v-app>
        <v-app-bar app ref="Top Bar" color="#cbb26b" density="compact">
            <v-app-bar-title> ATeam UI </v-app-bar-title>
        </v-app-bar>
        <v-main>
            <v-container class="d-flex flex-row" ref="Main Components">
                <RefButtonsComponent/>
                <StatusComponent/>
                <FieldComponent/>
            </v-container>
        </v-main>
    </v-app>
</template>


<script lang="ts">

import FieldComponent from './components/FieldComponent.vue'
import StatusComponent from './components/StatusComponent.vue'
import RefButtonsComponent from './components/RefButtonsComponent.vue'
import { provide } from 'vue'
import { defineComponent } from 'vue'

import { AppState } from '@/state'

export default {
    data() {
        return {
            state: new AppState(),
            renderConfig: {
                angle: 0,
                scale: 300, // Pixels per meter (in the rendering canvas)
                factor: .25 // Field Scaling Factor (applied to the canvas when it's added to the UI)
            }
        }
    },
    provide() { 
        return {
            state: this.state,
            renderConfig: this.renderConfig
        }
    },
    components: {
        FieldComponent,
        StatusComponent,
        RefButtonsComponent
    }
}
</script>
