# AWS DeepRacer workshop

### Setup:

#### Install Hugo:
On a mac:

`brew install hugo`

On Linux:
  - Download from the releases page: https://github.com/gohugoio/hugo/releases/tag/v0.46
  - Extract and save the executable to `/usr/local/bin`

On Windows:

If you are on a Windows machine and use Chocolatey for package management, you can install Hugo with the following one-liner:

`choco install hugo -confirm`

If you are on a Windows machine and use Scoop for package management, you can install Hugo with the following one-liner:

`scoop install hugo`

#### Clone this repo:
From wherever you checkout repos:
`git clone git@github.com:jerwallace/aws-robotics-day.git` (or your fork)

#### Clone the theme submodule:
`cd eksworkshop`

`git submodule init` ;
`git submodule update`

#### Install Node.js and npm:
You can follow instructions from npm website: https://www.npmjs.com/get-npm

#### Install node packages:
`npm install`

#### Run Hugo locally:
`npm run server`
or
`npm run drafts` to see stubbed in draft pages.

`npm run build` will build your content locally and output to `./public/`

`npm run test` will test the built content for bad links

#### View Hugo locally:
Visit http://localhost:8080/ to see the site.

#### Making Edits:
As you save edits to a page, the site will live-reload to show your changes.

