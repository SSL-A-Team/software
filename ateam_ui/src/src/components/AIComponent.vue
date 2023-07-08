<template>
    <v-card variant="outlined" style="white-space: nowrap; overflow: hidden">
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
            let object = JSON.parse(this.state.world.ai.description);
            if (object && this.state.world.ai.description != "{}") {
                return object;
            }

            object = {};
            object[this.state.world.ai.name] = "";
            return object;
        }
    },
    watch: {
        getAIDescription: {
            handler() {
            },
            deep: true
        },
    },
    components: {
        AIRecursiveComponent
    }
}
</script>
