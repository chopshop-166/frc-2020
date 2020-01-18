package com.chopshop166

import org.gradle.api.Plugin
import org.gradle.api.Project
import org.gradle.api.tasks.bundling.Jar
import org.gradle.api.java.archives.internal.DefaultManifest

class ChopShopPlugin implements Plugin<Project> {

    ChopShopExtension extension = null

    void apply(Project project) {
        project.configure(project) {
            // Make a fat JAR
            tasks.withType(Jar).configureEach { Jar jar ->
                jar.from { configurations.runtimeClasspath.collect { it.isDirectory() ? it : zipTree(it) } }
            }

            // Get the Chop Shop configuration
            extension = project.extensions.create('chopshop', ChopShopExtension)

            // Add chopshop libraries
            def deps = configurations.implementation.dependencies
            deps.add(dependencies.create("com.chopshop166:chopshoplib:${extension.version}"))
            deps.add(dependencies.create("com.chopshop166:lightdrive-mirror:${extension.lightdriveVersion}"))
        }
    }

    static Closure manifest(String robotMainClass) {
        def runCommand = { String... args ->
            return args.execute().text.trim()
        }

        def getGitHash = { -> runCommand "git", "describe", "--always" }
        def getGitBranch = { -> runCommand "git", "rev-parse", "--abbrev-ref", "HEAD" }
        def getGitFilesChanged = { -> runCommand ("git", "diff", "--name-only", "HEAD").replaceAll("\n", ", ") }

        return { DefaultManifest mf -> 
            mf.attributes 'Main-Class': robotMainClass
            mf.attributes 'Git-Hash': getGitHash(), 'Git-Branch': getGitBranch(), 'Git-Files': getGitFilesChanged()
            def buildDate = new Date()
            mf.attributes 'Build-Time': buildDate.format("yyyy-MM-dd HH:mm:ss")
        }
    }
}