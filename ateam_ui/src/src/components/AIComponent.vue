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
import AIRecursiveComponent from "./AIRecursiveComponent.vue";

export default {
    inject: ['state'],
    mounted() {
    },
    computed: {
        getAIDescription: function() {
            let object = {};
            try {
                object[this.state.world.ai.name] = JSON.parse(this.state.world.ai.description);
            } catch(e) {
                if (e instanceof SyntaxError) {
                    console.log("Play description contained invalid JSON")
                }
            }

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
