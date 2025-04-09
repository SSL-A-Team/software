<template>
        {{ name }}

        <div v-if="isLeaf" :style="{'margin-left': spacing + 'px'}">
            <!-- Leaf: String/Number/Bool -->
            {{ node }}
        </div>

        <div v-if="isArray" :style="{'margin-left': spacing + 'px'}">
            <!-- Array -->
            <AIRecursiveComponent
                v-for="item in node"
                    :key= "item"
                    :name= "''"
                    :node= "item"
                    :spacing= "spacing + 5"
                    style="white-space: normal"
            />
        </div>

        <div v-if="!isArray && !isLeaf && isValid" :style="{'margin-left': spacing + 'px'}">
            <!-- Recursive Object -->
            <AIRecursiveComponent
                v-for="member in Object.getOwnPropertyNames(node)"
                    :key= "member"
                    :name= "member"
                    :node= "node[member]"
                    :spacing= "spacing + 5"
                    style="white-space: normal"
            />
        </div>
</template>

<script lang="ts">

export default {
    props: {
        node: {
            type: Object,
            required: true
        },
        name: {
            type: String,
            required: true
        },
        spacing: {
            type: Number,
            default: 5
        }
    },
    computed: {
        isValid: function() {
            return !(this.node === null || this.node === undefined)
        },
        isLeaf: function() {
            return !Array.isArray(this.node) &&
                (typeof this.node === "string" || typeof this.node=== "number" || typeof this.node === "boolean")
        },
        isArray: function() {
            return Array.isArray(this.node)
        },
        getMargin: function() {
            return "margin-left: " + this.spacing + "px";
        }
    }
}
</script>
