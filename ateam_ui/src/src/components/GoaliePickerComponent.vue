<template>
  <v-container class="d-flex flex-column">
    <b>Goalie</b>
    <p>ID: {{ this.state.getGoalie() }}</p>
    <v-select v-model="selectedItem" label="set ID" class="flex-grow-0 align-end" :items="Array.from({length: 16}, (value, index) => index)" density="compact" variant="solo" @update:modelValue="setGoalieId"/>
  </v-container>
  <v-alert title="Set Goalie Failed" :text="this.state.goalie_service_status[1]" type="error" density="compact" :model-value="alert"></v-alert>
</template>

<script lang="ts">

export default {
  inject: ['state'],
  data() {
    return {
      alert: false,
      selectedItem: null
    }
  },
  mounted() {
  },
  methods: {
    setGoalieId(new_id) {
      this.state.setGoalie(new_id);
    }
  },
  computed: {
    goalie_service_status: function() {
      return this.state.goalie_service_status
    }
  },
  watch: {
    goalie_service_status: {
      handler() {
        if (this.state.goalie_service_status[0]) {
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