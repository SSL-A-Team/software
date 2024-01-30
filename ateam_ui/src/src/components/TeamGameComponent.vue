<template>
	<v-card
		class="pa-1 d-inline-block"
		variant="outlined"
		:style="{ color: this.team.color }"
		style="max-width: 600px; white-space: nowrap"
	>
		<p style="max-width: 300px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap">
			Team: {{ this.team.name }}
		</p>
		<p style="text-align: center; white-space: nowrap">Score: {{ this.team.score }}</p>
	</v-card>
</template>

<script lang="ts">
import { ref, inject } from 'vue';
import { Referee, GameProperty } from '@/referee';

export default {
	inject: ['state'],
	props: ['team'],
	computed: {
		getRefState: function () {
			return this.state.world.referee;
		}
	},
	watch: {
		getRefState: {
			handler() {},
			deep: true
		}
	}
};
</script>
