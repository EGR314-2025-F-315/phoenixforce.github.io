# Phoenix Force Project Site

This repository hosts the Phoenix Force website for the EGR314 course. The site documents our TerraGuard concept, team organization, assignment deliverables, and support materials in a single GitHub Pages deployment that faculty and external reviewers can access quickly.

## Live Site
- Production: https://egr314-2025-f-315.github.io/phoenixforce.github.io/
- Repository: https://github.com/EGR314-2025-F-315/phoenixforce.github.io

## Project Highlights
- Presents TerraGuard, an environmental monitoring concept aimed at detecting wildfire precursors.
- Publishes high quality HTML summaries for team charter, product framing, grading checklists, and supporting references.
- Embeds assignment artifacts such as YouTube presentations and PDF reports while keeping core content in web native formats.
- Uses Tailwind CSS and daisyUI via CDN for modern visuals without a build pipeline.

## Repository Layout
- `index.html` landing page with hero messaging, project highlights, and navigation.
- `team-organization/` charter, product mission, and team process documentation.
- `assignments/design-ideation/` Assignment 1 hub with embedded presentation and downloadable reports.
- `appendix/` supplemental notes, decisions, and organization artifacts.
- `assets/` shared media, PDFs, and custom CSS or JavaScript referenced across pages.

## Working Locally
1. Clone the repository: `git clone https://github.com/EGR314-2025-F-315/phoenixforce.github.io.git`
2. Start a lightweight server from the project root, for example `python3 -m http.server 8000`
3. Visit `http://localhost:8000/` to browse the site exactly as it will render on GitHub Pages.

## Updating Content
- Edit the relevant HTML page directly to update copy, tables, or embedded media. Keep headings and section structure consistent so PDF exports remain clean.
- Place new images, PDFs, or videos under the `assets/` directory and link to them with absolute paths that include `/phoenixforce.github.io/` so they work on GitHub Pages.
- For additional assignments, duplicate the `assignments/design-ideation/` folder, adjust the navigation links, and update the embedded media IDs.
- Review each page in both desktop and mobile breakpoints. Tailwind utility classes are already tuned for responsive layouts, so small adjustments usually only require editing class lists.

## Styling and Scripts
- Tailwind CSS and daisyUI are loaded from CDNs inside each page head. No build tooling or Node dependencies are required.
- `assets/styles.css` provides the NOVA theme used by the Resources page. Reuse its tokens when designing additional static pages for visual consistency.
- `assets/script.js` adds smooth scrolling for in-page anchor links. Extend this file if light client-side behavior is needed.

## Deployment Notes
- GitHub Pages serves the main branch automatically. Pushes to `main` are deployed once Actions finish processing.
- Because the site lives in an organization account, asset links use the fully qualified prefix `https://egr314-2025-f-315.github.io/phoenixforce.github.io/`. Maintain this convention for any new static files to avoid broken references.
- Use `gh-pages` previews or a local server to validate embedded YouTube links and PDF accessibility before merging changes.

## Maintenance Checklist
- Validate navigation links and external resources at the start of each sprint.
- Re-run accessibility scans (for example Lighthouse in Chrome) after significant content changes.
- Keep PDFs updated when new report revisions ship and remove outdated versions when they are no longer referenced.
- Coordinate edits through pull requests so reviewers can comment before content is published.

## Team
Phoenix Force is a student team in EGR314. Questions about the site or requests for updates can be directed to the current web maintainer documented in the course workspace.
