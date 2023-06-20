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
import { defineComponent } from 'vue'

import { AppState } from '@/state'


export default {
    data() {
        return {
            intervalId: null,
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
    methods: {
        update: function() {
            if (this.state.history.length < 10) {
                this.state.history.push(this.state.world);
            }

            // update components
            this.$refs.mainField.update();
        }
    },
    beforeUnmount() {
        clearInterval(this.intervalId);
    },
    created() {
        this.intervalId = setInterval(this.update, 1000);
    },
    components: {
        FieldComponent,
        StatusComponent,
        RefButtonsComponent
    }
}
</script>
