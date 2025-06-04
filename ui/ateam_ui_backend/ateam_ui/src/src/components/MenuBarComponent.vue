<template>
    <v-menu style="color: rgb(var(--v-theme-ateam-color))" location="bottom"> 
        <template v-slot:activator="{ props }">
            <v-btn class="px-0 ml-3" v-bind="props">
                File
            </v-btn>
        </template>
        <v-list style="background-color: rgb(var(--v-theme-ateam-color));">
            <v-list-item @click.stop=savePlaybook()> Save Playbook </v-list-item>
            <v-list-item @click.stop=loadPlaybook()> Load Playbook </v-list-item>
        </v-list>
    </v-menu>
    <v-menu style="color: rgb(var(--v-theme-ateam-color))" location="bottom"> 
        <template v-slot:activator="{ props }">
            <v-btn class="px-0 ml-3" v-bind="props">
                View
            </v-btn>
        </template>
        <v-list style="background-color: rgb(var(--v-theme-ateam-color));">
            <v-checkbox 
                label="Use Kenobi World Topic" 
                v-model="useKenobiTopic"
                @change="setUseKenobiTopic()"
                density="compact"
                hide-details
                class="px-2"
            />
            <v-list-item link>
                Rotate Field
                <template v-slot:append>
                    <v-icon icon="mdi-menu-right" size="small"></v-icon>
                </template>
                <v-menu activator="parent" :open-on-focus="false" open-on-hover submenu location="right"> 
                    <v-list style="background-color: rgb(var(--v-theme-ateam-color));">
                        <v-list-item v-for="angle of [0, 90, 180, 270]" :key="'rotateField' + angle" @click.stop=setFieldRotation(angle)>
                            <v-list-item-title> {{ angle }} </v-list-item-title>
                        </v-list-item>
                    </v-list>
                </v-menu>
            </v-list-item>
            <v-list-item link>
                Select Theme
                <template v-slot:append>
                    <v-icon icon="mdi-menu-right" size="small"></v-icon>
                </template>
                <v-menu activator="parent" :open-on-focus="false" open-on-hover submenu location="right"> 
                    <v-list style="background-color: rgb(var(--v-theme-ateam-color));">
                        <v-list-item v-for="[internalName, theme] of themeList" :key="'setTheme' + theme.variables.name" @click.stop=setTheme(internalName)>
                            <v-list-item-title> {{ theme.variables.name }} </v-list-item-title>
                        </v-list-item>
                    </v-list>
                </v-menu>
            </v-list-item>
        </v-list>
    </v-menu>
</template>

<script lang="ts">
import { useTheme } from "vuetify";
import "@mdi/font/css/materialdesignicons.css";
import { arrayBuffer } from "stream/consumers";

export default {
    inject: ['state'],
    data() {
        return {
            useKenobiTopic: true,
            globalTheme: useTheme()
        }
    },
    computed: {
        themeList: function(): Map<string, any> {
            const themes = new Map<string, any>();
            for (const internalName of Object.keys(this.globalTheme.themes)) {
                const theme = this.globalTheme.themes[internalName];
                if (theme.variables && theme.variables.name) {
                    themes.set(internalName, theme);
                }
            }

            return themes;
        }
    },
    methods: {
        savePlaybook() {
            console.log("not implemented yet")
        },
        loadPlaybook() {
            console.log("not implemented yet")
        },
        setUseKenobiTopic() {
            console.log("set kenobi: ", this.useKenobiTopic)
            this.state.setUseKenobiTopic(this.useKenobiTopic);
        },
        setFieldRotation(angle: number) {
            console.log("set field rotation: ", angle);
            this.state.renderConfig.angle = angle;
        },
        setTheme(themeName: string) {
            // TODO: This doesn't seem to work
            this.globalTheme.name = themeName;
        }
    }
}
</script>