# Nvblox Foxglove

Nvblox Foxglove provides tools to visualize nvblox ROS messages in
[Foxglove Studio](https://github.com/foxglove/studio).


## Installation

This extension uses the `npm` package manager to install dependencies and run
build scripts. All `npm` commands have to be run from the `nvblox_foxglove` folder.

> This extension is known to work with node versions `v10.2.3` and `v20.10.0`. Other
(particularly older) versions fail during the installation steps. To update node
run, we recommend using `nvm`, to upgrade node.
> ```
> curl https://raw.githubusercontent.com/creationix/nvm/master/install.sh | bash
> nvm install 20.10.0
> nvm use 20.10.0
> ```

To install extension dependencies, run:

```npm install ```

To build and install the extension into your local Foxglove Studio desktop app, run:

```npm run local-install ```

To test the extensions open the `Foxglove Studio` desktop (or `ctrl-R` to refresh if it is already
open). Your extension is installed and available within the app. You can open `Foxglove Studio` from
a terminal with the command `foxglove-studio`.


## Development

If you want to view debugging messages printed by `console.log(...)` from the extension make sure to
run `export ELECTRON_ENABLE_LOGGING=1` in the terminal where you open `Foxglove Studio`.


## Packaging

Extensions are packaged into `.foxe` files. These files contain the metadata (package.json) and the
build code for the extension.

The following command will package the extension into a `.foxe` file in the local directory.

```npm run package ```


## Publishing

You can publish the extension for the public marketplace or privately for your organization.

See documentation here: https://foxglove.dev/docs/studio/extensions/publish#packaging-your-extension
