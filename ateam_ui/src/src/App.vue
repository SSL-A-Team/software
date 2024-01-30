<template>
	<v-app>
		<v-app-bar app ref="Top Bar" color="#cbb26b" density="compact">
			<v-app-bar-title> ATeam UI </v-app-bar-title>
		</v-app-bar>
		<v-main style="height: 100vh; width: 100vw">
			<v-container fluid class="w-100 h-100 pa-0 justify-space-between">
				<v-row class="ma-0 w-100 h-100">
					<v-col cols="1" v-if="!this.state.comp" class="pa-1 h-100" style="min-width: 180px">
						<RefButtonsComponent />
					</v-col>
					<v-col cols="1" class="pa-1 h-100" style="min-width: 220px; max-width: 220px; overflow-y: scroll">
						<StatusComponent ref="robotStatus" />
					</v-col>
					<v-col align="center" class="h-100 px-1 pt-2">
						<GameStatusComponent ref="refStatus" />
						<FieldComponent ref="mainField" />
					</v-col>
					<v-col cols="2" class="pa-1 h-100" style="min-width: 220px">
						<AIComponent ref="AIStatus" />
					</v-col>
				</v-row>
			</v-container>
		</v-main>
	</v-app>
</template>

<script lang="ts">
import FieldComponent from './components/FieldComponent.vue';
import StatusComponent from './components/StatusComponent.vue';
import RefButtonsComponent from './components/RefButtonsComponent.vue';
import GameStatusComponent from './components/GameStatusComponent.vue';
import AIComponent from './components/AIComponent.vue';
import { provide } from 'vue';
import { defineComponent, toRaw } from 'vue';

import { AppState } from '@/state';

export default {
	data() {
		return {
			intervalIds: [],
			state: new AppState(),
			renderConfig: {
				angle: 0,
				scale: 75, // Pixels per meter (in the rendering canvas)
				factor: 1 // Field Scaling Factor (applied to the canvas when it's added to the UI)
			}
		};
	},
	provide() {
		return {
			state: this.state,
			renderConfig: this.renderConfig
		};
	},
	methods: {
		// Renders field at 100fps
		updateField: function () {
			this.$refs.mainField.update();
		},
		updateStatus: function () {
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
		AIComponent
	}
};
</script>
