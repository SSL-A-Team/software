<template>
  <v-container class="d-flex flex-column">
    <b>Goalie</b>
    <p>ID: {{ state.getGoalie() }}</p>
    <v-select v-model="selectedItem" label="set ID" class="flex-grow-0 align-end" :items="Array.from({length: 16}, (value, index) => index)" density="compact" variant="solo" @update:modelValue="setGoalieId"/>
  </v-container>
  <v-alert title="Set Goalie Failed" :text="state.goalieServiceStatus[1]" type="error" density="compact" :model-value="alert"></v-alert>
</template>

<script lang="ts">
import { inject } from "vue";
import { AppState } from "@/state";

export default {
  inject: ['state'],
  data() {
    return {
      alert: false,
      selectedItem: null,
      state: inject('state') as AppState
    }
  },
  mounted() {
  },
  methods: {
    setGoalieId(newId: number) {
      this.state.setGoalie(newId);
    }
  },
  computed: {
    goalieServiceStatus: function() {
      return this.state.goalieServiceStatus
    }
  },
  watch: {
    goalieServiceStatus: {
      handler() {
        if (this.state.goalieServiceStatus[0]) {
          this.alert = false;
        } else {
          this.alert = true;

          const state = this;
          setTimeout(function() {
            state.alert = false;

            const goalie = parseInt(state.state.getGoalie());
            if (isNaN(goalie)) {
              state.selectedItem = null;
            } else {
              state.selectedItem = goalie;
            }
          }, 4000);
        }
      },
      deep: true
    }
  },
  components: {
  }
}

</script>