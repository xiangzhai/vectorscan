pipeline {
  agent {
    node {
      label 'x86'
    }

  }
  stages {
    stage('Configure Release, SSE') {
      agent {
        node {
          label 'x86'
        }

      }
      steps {
        sh 'mkdir build-release-SSE &&  cmake -DCMAKE_BUILD_TYPE=Release   -C build-release-SSE'
      }
    }

    stage('Build Release, SSE') {
      steps {
        sh 'cd build-release-SSE &&  make -j4'
      }
    }

    stage('Test Release, SSE') {
      steps {
        sh 'build-release-SSE/bin/unit-hyperscan'
      }
    }

  }
}