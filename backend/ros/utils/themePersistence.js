// backend/utils/themePersistence.js
const fs = require('fs-extra');
const path = require('path');

class ThemePersistence {
    constructor() {
        this.themePath = path.join(__dirname, '../storage/theme.json');
        this.defaultTheme = { isDark: false };
    }

    async getTheme() {
        try {
            if (await fs.pathExists(this.themePath)) {
                const theme = await fs.readJson(this.themePath);
                return theme;
            }
            return this.defaultTheme;
        } catch (error) {
            console.error('Error loading theme:', error);
            return this.defaultTheme;
        }
    }

    async setTheme(isDark) {
        try {
            const theme = { isDark, updatedAt: new Date().toISOString() };
            await fs.ensureDir(path.dirname(this.themePath));
            await fs.writeJson(this.themePath, theme, { spaces: 2 });
            return theme;
        } catch (error) {
            console.error('Error saving theme:', error);
            throw error;
        }
    }
}

// Create singleton instance
const themePersistence = new ThemePersistence();
module.exports = themePersistence;