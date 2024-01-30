<template>
	<v-row class="ma-0 pa-0 justify-space-between" style="height: 160px">
		<v-col cols="1" class="pa-1 h-100" style="min-width: 120px">
			<p class="w-100">Goalie ID: {{ this.state.getGoalie() }}</p>
			<v-select
				label="Set ID"
				class="w-100"
				style="min-width: 110px; max-width: 150px"
				:items="Array.from({ length: 16 }, (value, index) => index)"
				density="compact"
				variant="solo"
				@update:modelValue="setGoalieId"
			/>
		</v-col>
		<v-col
			v-if="this.state.world.referee.blue.name || this.state.world.referee.yellow.name"
			class="pa-2 h-100"
			style="overflow-x: auto; overflow-y: hidden; white-space: nowrap; scroll-snap-align: center"
		>
			<v-card
				variant="outlined"
				class="ma-2 pa-2 d-inline-block"
				:style="{ color: stageProperty.color }"
				style="max-width: 400px; white-space: nowrap"
			>
				{{ this.stageProperty.name }}
			</v-card>
			<v-card
				variant="outlined"
				class="ma-2 pa-2 d-inline-block"
				:style="{ color: commandProperty.color }"
				style="max-width: 400px; white-space: nowrap"
			>
				{{ this.commandProperty.name }}
			</v-card>
			<br />
			<TeamGameComponent v-bind:team="this.state.world.referee.blue" style="background: #0066ff" />
			<div class="px-1 d-inline-block" style="height: 60%; vertical-align: middle; white-space: nowrap">vs</div>
			<TeamGameComponent v-bind:team="this.state.world.referee.yellow" style="background: #ffee00" />
		</v-col>
		<v-col cols="1" class="pa-1 h-100" style="min-width: 120px"> </v-col>
	</v-row>
</template>

<script lang="ts">
import { ref, inject } from 'vue';
import { Referee, GameProperty } from '@/referee';
import TeamGameComponent from './TeamGameComponent.vue';

export default {
	inject: ['state'],
	mounted() {},
	methods: {
		setGoalieId(new_id) {
			this.state.setGoalie(new_id);
		}
	},
	computed: {
		getRefState: function () {
			return this.state.world.referee;
		},
		stageProperty: function () {
			return this.state.world.referee.getStageProperty();
		},
		commandProperty: function () {
			return this.state.world.referee.getCommandProperty();
		}
	},
	watch: {
		getRefState: {
			handler() {},
			deep: true
		}
	},
	components: {
		TeamGameComponent,
		TeamGameComponentInline
	}
};
</script>
