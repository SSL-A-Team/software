<template>
    <v-card variant="outlined" style="white-space: nowrap; overflow: visible">
        <div style="margin: 5px">
            <AIRecursiveComponent v-for="member of Object.getOwnPropertyNames(getAIDescription)"
                :key="member"
                :name="member"
                :node="getAIDescription[member]"
                :spacing=5
                />
        </div>
    </v-card>
</template>

<script lang="ts">
import { ref, inject } from "vue";
import { AIState } from "@/AI";
import { Referee, GameStage, GameCommand} from "@/referee";
import AIRecursiveComponent from "./AIRecursiveComponent.vue";

export default {
    inject: ['state'],
    mounted() {
        console.dir(this.$el)
    },
    computed: {
        getAIDescription: function() {
            return JSON.parse(this.state.world.ai.description);
        }
    },
    watch: {
        getAIState: {
            handler() {
                this.$el.textContent = '';
            },
            deep: true
        },
    },
    components: {
        AIRecursiveComponent
    }
}
</script>
