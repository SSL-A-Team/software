<template>
    <v-container class="d-flex flex-column">
        <b>Field Half</b>
        <v-switch 
            label="Ignore - Field Half"
            density="compact"
            v-model="negative_half"
            @change="setNegative()"
        />
        <v-switch 
            label="Ignore + Field Half"
            density="compact"
            v-model="positive_half"
            @change="setPositive()"
        />
    </v-container>
</template>

<script lang="ts">

export default {
    inject: ['state'],
    data: function() {
        return {
            negative_half: false,
            positive_half: false
        }
    },
    methods: {
        setNegative: function() {
            if (this.negative_half && this.positive_half) {
                this.positive_half = false;
            }
        },
        setPositive: function() {
            if (this.negative_half && this.positive_half) {
                this.negative_half = false;
            }
        }
    },
    computed: {
    }
}
</script>
