import { useColorMode } from "@docusaurus/theme-common";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faGithub } from '@fortawesome/free-brands-svg-icons';
import guilherme from '@site/static/img/gui.jpg';

import React from "react";

const array = [
    {
        name: "Cecilia Gonçalves",
        link: "https://www.linkedin.com/in/cec%C3%ADlia-alonso-gon%C3%A7alves-3aa4bb271/",
        image: "https://media.licdn.com/dms/image/v2/D4E03AQHFDADl2nqTcA/profile-displayphoto-shrink_200_200/profile-displayphoto-shrink_200_200/0/1680660675815?e=1734566400&v=beta&t=pq11mDN0nJEsC4eOWKuX_qHa8u_tWVH7f6BesPd7HFw",
        github: "cecigonca",
        githubLink: "https://github.com/cecigonca",

    },
    {
        name: "Eduardo Santos",
        link: "https://www.linkedin.com/in/eduardo-henrique-dos-santos/",
        image: "https://media.licdn.com/dms/image/v2/D4D03AQEwG6Ov3yQQnw/profile-displayphoto-shrink_400_400/profile-displayphoto-shrink_400_400/0/1723207695428?e=1734566400&v=beta&t=ICXz9psFzSTxQ_GExVpy2fUUzwhoGmTw1JB85SRNVYo",
        github: "Edustn",
        githubLink: "https://github.com/Edustn",

    },
    {
        name: "Fernando Vasconcellos",
        link: "https://www.linkedin.com/in/fernando-vasconcellos-/",
        image: "https://media.licdn.com/dms/image/v2/D4D03AQEDNLTZO5WUdA/profile-displayphoto-shrink_400_400/profile-displayphoto-shrink_400_400/0/1728383447646?e=1734566400&v=beta&t=coTvxdzH0XyMC9p6DSoNDSb0x_Q0ykpT202PG9Zaysg",
        github: "ItsVasconcellos",
        githubLink: "https://github.com/ItsVasconcellos",

    },
    {
        name: "Gabriel Gallo",
        link: "https://www.linkedin.com/in/gabriel-gallo-m-coutinho-443809232/",
        image: "https://media.licdn.com/dms/image/v2/D4E03AQGQ_hxvNv8a2w/profile-displayphoto-shrink_400_400/profile-displayphoto-shrink_400_400/0/1665073284301?e=1734566400&v=beta&t=F5ZgX_Mddl_t3Eib5dagjOjYbDvV3yaT16wpKkJ28Ck",
        github: "GalloGGMC",
        githubLink: "https://github.com/GalloGGMC",

    },
    {
        name: "Guilherme Ferreira",
        link: "https://www.linkedin.com/in/guilherme-ferreira-linhares-8638411a1/",
        image: guilherme,
        github: "GuilhermeLinhares04",
        githubLink: "https://github.com/GuilhermeLinhares04",

    },
    {
        name: "José Alencar",
        link: "https://www.linkedin.com/in/josevalencar/",
        image: "https://media.licdn.com/dms/image/v2/D4D03AQGcVfLbFU12Ww/profile-displayphoto-shrink_400_400/profile-displayphoto-shrink_400_400/0/1714085847839?e=1734566400&v=beta&t=jkCj-Baw1mNiMZxcm3LTfh2x44S-VFdxwQT8HfeH5qE",
        github: "josevalencar",
        githubLink: "https://github.com/josevalencar",

    },
    {
        name: "Lídia Mariano",
        link: "https://www.linkedin.com/in/lidiamariano/",
        image: "https://media.licdn.com/dms/image/v2/D4D03AQGi2xctp-JiwA/profile-displayphoto-shrink_800_800/profile-displayphoto-shrink_800_800/0/1725480695754?e=1734566400&v=beta&t=ljI2frkyI5qReJLSSu4EbygA2TEMZa_N9LX-Ders6ZQ",
        github: "lidiamariano",
        githubLink: "https://github.com/lidiamariano",

    },
    {
        name: "Vitória Novaes",
        link: "https://www.linkedin.com/in/vitoria-novaes/",
        image: "https://media.licdn.com/dms/image/v2/D4D03AQEb7i7ITx0tOw/profile-displayphoto-shrink_800_800/profile-displayphoto-shrink_800_800/0/1678715409883?e=1734566400&v=beta&t=3xcUI73yTxeGVGlHvpwnWOkN3w0yhBUe7XNeldGVRCM",
        github: "vitorianovaesx",
        githubLink: "https://github.com/vitorianovaesx",

    },
];

const Card = ({ element }) => {
    const { isDarkTheme } = useColorMode();

    const cardStyle = {
        display: "flex",
        flexDirection: "row",
        alignItems: "center",
        paddingLeft: "1vw",
        paddingRight: "1vw",
        height: "100%",
        width: "100%",
    };
    console.log(element.image);
    return (
        <div style={cardStyle}>
            <img
                src={element.image}
                alt={element.name}
                style={{
                    width: "50%",
                    height: "100%",
                    marginRight: "1vw",
                    borderRadius: "4%",
                }}
            />
            <div>
                <a
                    href={element.link}
                    style={{
                        textDecoration: "none",
                        color: isDarkTheme ? "#4589ff" : "black",
                    }}
                >
                    <h3 style={{ margin: "1vh 0vw", textAlign: "left",  }}>
                        {element.name}
                    </h3>
                </a>
                <div style={{display: "flex"}}>
                    <FontAwesomeIcon icon={faGithub} size="xl" />
                    <p style={{ margin: "0vh 0.5vw" }}>
                        <a href={`${element.githubLink}`} style={{color: isDarkTheme ? "#4589ff" : "black"}}>{element.github}</a>
                    </p>
                </div>
            </div>
        </div>
    );
};

const Cards = () => {
    const { isDarkTheme } = useColorMode();

    const divStyle = {
        margin: "1vw 0.5vh",
        padding: "1vh 2vw",
        borderRadius: "4vh",
        width: "calc(28*2vw/2)",
        height: "28vh",
        backgroundColor: isDarkTheme ? "black" : "#f4f4f4",
        color: isDarkTheme ? "#fff" : "#000",
    };

    return (
        <div
            style={{
                display: "grid",
                gridTemplateColumns: "repeat(2, 1fr)", // 4 columns
                gridGap: "px",
                justifyContent: "center",
            }}
        >
            {array.map((element, index) => (
                <div style={divStyle}>
                    <Card key={index} element={element} />
                </div>
            ))}
        </div>
    );
};

export default Cards;
